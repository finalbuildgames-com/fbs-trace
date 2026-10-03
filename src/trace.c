/*
 * src/trace.c — FinalBuildSystems event-driven weapon trace.
 *
 * Implements include/fbs/trace.h version 0.1.0. C99, libm only (sqrt, ceil,
 * fabs, isfinite) plus malloc/free for the default allocator. No globals and
 * no static mutable state: every byte of state lives in the caller's structs
 * or in the single block owned by an fbs_trace_context.
 *
 * Numerics. The public API is float; everything in here is computed in double
 * and converted once at the boundary, so a native build and the WASM build
 * (both IEEE-754 double, both compiled with -ffp-contract=off) agree bit for
 * bit. Segment/segment closest points follow Ericson, Real-Time Collision
 * Detection, 5.1.9 (ClosestPtSegmentSegment) with an epsilon of 1e-12 on the
 * squared lengths and on the denominator, which is also the form a TypeScript
 * host game uses, so FBS_TRACE_MODE_SAMPLED with spacing 0.04 reproduces that
 * host's sampled capsule contact sample for sample.
 *
 * Two deliberate readings of the contract:
 *   - contact.distance is the closest-point distance between the weapon-axis
 *     point and the target core point. It is never negative; it is smaller
 *     than the combined radius when the weapon penetrates.
 *   - The combined radius R is taken from the interval-start poses
 *     (w0->radius + target0->radius) and is never interpolated, because the
 *     conservative-advancement bound L carries no radius-rate term.
 */
#include "fbs/trace.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Epsilon on squared lengths and on the segment/segment denominator. */
#define FBS_EPS 1e-12
/* Alignment every sub-array of the context block is carved on. */
#define FBS_ALIGN ((size_t)16)

/* ------------------------------------------------------------------------- */
/* Double vector helpers                                                     */
/* ------------------------------------------------------------------------- */

typedef struct fbs_dvec3 {
  double x, y, z;
} fbs_dvec3;

static fbs_dvec3 dv(double x, double y, double z) {
  fbs_dvec3 r;
  r.x = x;
  r.y = y;
  r.z = z;
  return r;
}

static fbs_dvec3 dv_of(fbs_vec3 v) { return dv((double)v.x, (double)v.y, (double)v.z); }

static fbs_vec3 dv_to(fbs_dvec3 v) {
  fbs_vec3 r;
  r.x = (float)v.x;
  r.y = (float)v.y;
  r.z = (float)v.z;
  return r;
}

static fbs_dvec3 dv_add(fbs_dvec3 a, fbs_dvec3 b) { return dv(a.x + b.x, a.y + b.y, a.z + b.z); }
static fbs_dvec3 dv_sub(fbs_dvec3 a, fbs_dvec3 b) { return dv(a.x - b.x, a.y - b.y, a.z - b.z); }
static fbs_dvec3 dv_scale(fbs_dvec3 a, double s) { return dv(a.x * s, a.y * s, a.z * s); }
static double dv_dot(fbs_dvec3 a, fbs_dvec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static double dv_len(fbs_dvec3 a) { return sqrt(dv_dot(a, a)); }

/* Lerp as a + t (b - a), the exact form the 4 cm hosts sample with. */
static fbs_dvec3 dv_lerp(fbs_dvec3 a, fbs_dvec3 b, double t) {
  return dv_add(a, dv_scale(dv_sub(b, a), t));
}

static double d_max(double a, double b) { return a > b ? a : b; }

/* NaN-safe clamp: anything that fails "v >= 0" (NaN included) becomes 0. */
static double clamp01(double v) {
  if (!(v >= 0.0)) return 0.0;
  if (v > 1.0) return 1.0;
  return v;
}

/* ------------------------------------------------------------------------- */
/* Validation                                                                */
/* ------------------------------------------------------------------------- */

static int fbs_finite(float v) {
  double d = (double)v;
  return isfinite(d) != 0;
}

static int vec_ok(fbs_vec3 v) { return fbs_finite(v.x) && fbs_finite(v.y) && fbs_finite(v.z); }

static int radius_ok(float r) { return fbs_finite(r) && r >= 0.0f; }

fbs_trace_status fbs_trace_bind_capsule(fbs_vec3 a,fbs_vec3 b,fbs_vec3 oa,fbs_vec3 ob,float radius,fbs_trace_capsule *out) {
  if(!out||!vec_ok(a)||!vec_ok(b)||!vec_ok(oa)||!vec_ok(ob)||!radius_ok(radius))return FBS_TRACE_E_INVALID;
  fbs_trace_capsule next={{a.x+oa.x,a.y+oa.y,a.z+oa.z},{b.x+ob.x,b.y+ob.y,b.z+ob.z},radius};
  if(!vec_ok(next.a)||!vec_ok(next.b))return FBS_TRACE_E_INVALID;
  *out=next;return FBS_TRACE_OK;
}


static int weapon_ok(const fbs_trace_weapon *w) {
  return w != NULL && vec_ok(w->a) && vec_ok(w->b) && radius_ok(w->radius);
}

static int capsule_ok(const fbs_trace_capsule *c) {
  return c != NULL && vec_ok(c->a) && vec_ok(c->b) && radius_ok(c->radius);
}

static int sphere_ok(const fbs_trace_sphere *s) {
  return s != NULL && vec_ok(s->center) && radius_ok(s->radius);
}

static int policy_ok(const fbs_trace_policy *p) {
  if (p == NULL) return 0;
  if (p->mode != (int)FBS_TRACE_MODE_SAMPLED && p->mode != (int)FBS_TRACE_MODE_ADVANCE) return 0;
  if (p->max_steps < 2u) return 0;
  if (p->mode == (int)FBS_TRACE_MODE_SAMPLED)
    return fbs_finite(p->spacing) && p->spacing > 0.0f;
  return fbs_finite(p->tolerance) && p->tolerance > 0.0f;
}

/* ------------------------------------------------------------------------- */
/* Closest points, Ericson RTCD 5.1.9                                        */
/* ------------------------------------------------------------------------- */

/* Closest points between segment p1..q1 (the weapon) and p2..q2 (the target
 * core). Handles point/point, point/segment, parallel and collinear inputs
 * without dividing by zero; every quotient is clamped, so no NaN escapes even
 * if a caller slipped a denormal past validation. Returns the distance. */
static double closest_seg_seg(fbs_dvec3 p1, fbs_dvec3 q1, fbs_dvec3 p2, fbs_dvec3 q2,
                              double *out_s, double *out_t, fbs_dvec3 *out_c1, fbs_dvec3 *out_c2) {
  fbs_dvec3 d1 = dv_sub(q1, p1);
  fbs_dvec3 d2 = dv_sub(q2, p2);
  fbs_dvec3 r = dv_sub(p1, p2);
  double a = dv_dot(d1, d1); /* squared length of the weapon segment */
  double e = dv_dot(d2, d2); /* squared length of the target axis */
  double f = dv_dot(d2, r);
  double s = 0.0;
  double t = 0.0;

  if (a <= FBS_EPS && e <= FBS_EPS) {
    /* point vs point */
    s = 0.0;
    t = 0.0;
  } else if (a <= FBS_EPS) {
    /* point weapon (fist) vs segment */
    s = 0.0;
    t = clamp01(f / e);
  } else {
    double c = dv_dot(d1, r);
    if (e <= FBS_EPS) {
      /* segment vs point target (sphere) */
      t = 0.0;
      s = clamp01(-c / a);
    } else {
      double b = dv_dot(d1, d2);
      double denom = a * e - b * b; /* always >= 0 */
      /* Parallel and collinear segments leave denom at (numerically) zero;
       * pick s = 0 and let the t fix-ups below place the pair. */
      s = denom > FBS_EPS ? clamp01((b * f - c * e) / denom) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-c / a);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((b - c) / a);
      } else if (!(t >= 0.0)) {
        t = 0.0; /* NaN guard; unreachable for validated finite inputs */
      }
    }
  }

  *out_c1 = dv_add(p1, dv_scale(d1, s));
  *out_c2 = dv_add(p2, dv_scale(d2, t));
  *out_s = s;
  *out_t = t;
  return dv_len(dv_sub(*out_c1, *out_c2));
}

/* ------------------------------------------------------------------------- */
/* Sweep core                                                                */
/* ------------------------------------------------------------------------- */

typedef struct fbs_sweep {
  fbs_dvec3 a0, b0, a1, b1; /* weapon endpoints at the interval ends */
  fbs_dvec3 c0, d0, c1, d1; /* target core endpoints (sphere: c == d) */
  double radius;            /* combined radius R */
  double travel;            /* max weapon endpoint travel + max target endpoint travel */
} fbs_sweep;

typedef struct fbs_sample {
  double dist, k, u;
  fbs_dvec3 point, target_point;
} fbs_sample;

static void sweep_setup(fbs_sweep *s, const fbs_trace_weapon *w0, const fbs_trace_weapon *w1,
                        fbs_dvec3 c0, fbs_dvec3 d0, fbs_dvec3 c1, fbs_dvec3 d1,
                        double target_radius) {
  s->a0 = dv_of(w0->a);
  s->b0 = dv_of(w0->b);
  s->a1 = dv_of(w1->a);
  s->b1 = dv_of(w1->b);
  s->c0 = c0;
  s->d0 = d0;
  s->c1 = c1;
  s->d1 = d1;
  s->radius = (double)w0->radius + target_radius;
  s->travel = d_max(dv_len(dv_sub(s->a1, s->a0)), dv_len(dv_sub(s->b1, s->b0))) +
              d_max(dv_len(dv_sub(s->c1, s->c0)), dv_len(dv_sub(s->d1, s->d0)));
}

static void sweep_eval(const fbs_sweep *s, double t, fbs_sample *out) {
  fbs_dvec3 a = dv_lerp(s->a0, s->a1, t);
  fbs_dvec3 b = dv_lerp(s->b0, s->b1, t);
  fbs_dvec3 c = dv_lerp(s->c0, s->c1, t);
  fbs_dvec3 d = dv_lerp(s->d0, s->d1, t);
  out->dist = closest_seg_seg(a, b, c, d, &out->k, &out->u, &out->point, &out->target_point);
}

static void sweep_fill(const fbs_sweep *s, double t, const fbs_sample *sm, unsigned steps_used,
                       fbs_trace_contact *out) {
  /* Material point of the weapon at parameter k, and of the target core at u. */
  fbs_dvec3 wm = dv_add(dv_scale(dv_sub(s->a1, s->a0), 1.0 - sm->k),
                        dv_scale(dv_sub(s->b1, s->b0), sm->k));
  fbs_dvec3 tm = dv_add(dv_scale(dv_sub(s->c1, s->c0), 1.0 - sm->u),
                        dv_scale(dv_sub(s->d1, s->d0), sm->u));
  memset(out, 0, sizeof *out);
  out->contact = 1;
  out->time = (float)t;
  out->weapon_param = (float)sm->k;
  out->target_param = (float)sm->u;
  out->distance = (float)sm->dist;
  out->point = dv_to(sm->point);
  out->target_point = dv_to(sm->target_point);
  if (sm->dist >= FBS_EPS)
    out->normal = dv_to(dv_scale(dv_sub(sm->point, sm->target_point), 1.0 / sm->dist));
  out->motion = dv_to(wm);
  out->relative_motion = dv_to(dv_sub(wm, tm));
  out->steps_used = steps_used;
}

static void sweep_miss(unsigned steps_used, fbs_trace_contact *out) {
  memset(out, 0, sizeof *out);
  out->steps_used = steps_used;
}

/* Runs the policy over the interval. `out` may be NULL when only the status is
 * wanted (fbs_trace_test probes for FBS_TRACE_E_CAPACITY before it writes any
 * hit, so a capacity failure leaves the caller's array untouched). */
static fbs_trace_status sweep_run(const fbs_sweep *s, const fbs_trace_policy *p,
                                  fbs_trace_contact *out) {
  fbs_sample sm;
  if (p->mode == (int)FBS_TRACE_MODE_SAMPLED) {
    /* steps = max(1, ceil(travel / spacing)); samples at i/steps, i = 0..steps.
     * The step count is compared in double so an absurd spacing cannot wrap. */
    double dsteps = ceil(s->travel / (double)p->spacing);
    unsigned steps, i;
    if (!(dsteps >= 1.0)) dsteps = 1.0;
    if (dsteps + 1.0 > (double)p->max_steps) return FBS_TRACE_E_CAPACITY;
    steps = (unsigned)dsteps;
    for (i = 0; i <= steps; ++i) {
      double t = (double)i / (double)steps;
      sweep_eval(s, t, &sm);
      if (sm.dist <= s->radius) {
        if (out != NULL) sweep_fill(s, t, &sm, i + 1u, out);
        return FBS_TRACE_CONTACT;
      }
    }
    if (out != NULL) sweep_miss(steps + 1u, out);
    return FBS_TRACE_OK;
  }

  /* Conservative advancement. |d/dt separation| <= travel, so advancing by
   * separation / travel can never step over a contact. */
  {
    double t = 0.0;
    double tol = (double)p->tolerance;
    unsigned iter = 0;
    for (; iter < p->max_steps; ++iter) {
      double sep;
      sweep_eval(s, t, &sm);
      sep = sm.dist - s->radius;
      if (sep <= tol) {
        if (out != NULL) sweep_fill(s, t, &sm, iter + 1u, out);
        return FBS_TRACE_CONTACT;
      }
      if (!(s->travel > 0.0)) break; /* no relative motion: nothing to advance */
      t += sep / s->travel;
      if (!(t <= 1.0)) break; /* past the interval (or non-finite): clear */
    }
    if (iter >= p->max_steps) return FBS_TRACE_E_CAPACITY;
    if (out != NULL) sweep_miss(iter + 1u, out);
    return FBS_TRACE_OK;
  }
}

/* ------------------------------------------------------------------------- */
/* Presets, names, version                                                   */
/* ------------------------------------------------------------------------- */

const char *fbs_trace_status_name(int status) {
  switch (status) {
    case FBS_TRACE_OK: return "ok";
    case FBS_TRACE_CONTACT: return "contact";
    case FBS_TRACE_E_INVALID: return "invalid";
    case FBS_TRACE_E_CAPACITY: return "capacity";
    case FBS_TRACE_E_MEMORY: return "memory";
    case FBS_TRACE_E_STATE: return "state";
    case FBS_TRACE_E_NOT_FOUND: return "not_found";
    case FBS_TRACE_E_FULL: return "full";
    case FBS_TRACE_E_DEDUP: return "dedup";
    default: return "unknown";
  }
}

fbs_trace_policy fbs_trace_policy_compat_4cm(void) {
  fbs_trace_policy p;
  p.mode = (int)FBS_TRACE_MODE_SAMPLED;
  p.spacing = 0.04f;
  p.tolerance = 1e-4f;
  p.max_steps = 1024u;
  return p;
}

fbs_trace_policy fbs_trace_policy_default(void) {
  fbs_trace_policy p;
  p.mode = (int)FBS_TRACE_MODE_ADVANCE;
  p.spacing = 0.04f;
  p.tolerance = 1e-4f;
  p.max_steps = 64u;
  return p;
}

fbs_trace_config fbs_trace_config_default(void) {
  fbs_trace_config c;
  c.max_weapons = 4u;
  c.max_windows = 8u;
  c.max_struck = 64u;
  c.dedup = (int)FBS_TRACE_DEDUP_TARGET_ZONE;
  c.policy = fbs_trace_policy_default();
  return c;
}

unsigned fbs_trace_version(void) { return (unsigned)FBS_TRACE_VERSION; }

/* ------------------------------------------------------------------------- */
/* Stateless sweeps                                                          */
/* ------------------------------------------------------------------------- */

fbs_trace_status fbs_trace_sweep_sphere(const fbs_trace_weapon *w0, const fbs_trace_weapon *w1,
                                        const fbs_trace_sphere *s0, const fbs_trace_sphere *s1,
                                        const fbs_trace_policy *policy, fbs_trace_contact *out) {
  fbs_sweep s;
  fbs_dvec3 c0, c1;
  if (out == NULL || !weapon_ok(w0) || !weapon_ok(w1) || !sphere_ok(s0) || !policy_ok(policy))
    return FBS_TRACE_E_INVALID;
  if (s1 != NULL && !sphere_ok(s1)) return FBS_TRACE_E_INVALID;
  c0 = dv_of(s0->center);
  c1 = s1 != NULL ? dv_of(s1->center) : c0;
  sweep_setup(&s, w0, w1, c0, c0, c1, c1, (double)s0->radius);
  return sweep_run(&s, policy, out);
}

fbs_trace_status fbs_trace_sweep_capsule(const fbs_trace_weapon *w0, const fbs_trace_weapon *w1,
                                         const fbs_trace_capsule *c0, const fbs_trace_capsule *c1,
                                         const fbs_trace_policy *policy, fbs_trace_contact *out) {
  fbs_sweep s;
  if (out == NULL || !weapon_ok(w0) || !weapon_ok(w1) || !capsule_ok(c0) || !policy_ok(policy))
    return FBS_TRACE_E_INVALID;
  if (c1 != NULL && !capsule_ok(c1)) return FBS_TRACE_E_INVALID;
  sweep_setup(&s, w0, w1, dv_of(c0->a), dv_of(c0->b),
              c1 != NULL ? dv_of(c1->a) : dv_of(c0->a),
              c1 != NULL ? dv_of(c1->b) : dv_of(c0->b), (double)c0->radius);
  return sweep_run(&s, policy, out);
}

fbs_trace_status fbs_trace_closest(const fbs_trace_weapon *w, const fbs_trace_capsule *c,
                                   fbs_trace_contact *out) {
  fbs_sample sm;
  double r;
  if (out == NULL || !weapon_ok(w) || !capsule_ok(c)) return FBS_TRACE_E_INVALID;
  sm.dist = closest_seg_seg(dv_of(w->a), dv_of(w->b), dv_of(c->a), dv_of(c->b), &sm.k, &sm.u,
                            &sm.point, &sm.target_point);
  r = (double)w->radius + (double)c->radius;
  memset(out, 0, sizeof *out);
  out->contact = sm.dist <= r ? 1 : 0;
  out->time = 0.0f;
  out->weapon_param = (float)sm.k;
  out->target_param = (float)sm.u;
  out->distance = (float)sm.dist;
  out->point = dv_to(sm.point);
  out->target_point = dv_to(sm.target_point);
  if (sm.dist >= FBS_EPS)
    out->normal = dv_to(dv_scale(dv_sub(sm.point, sm.target_point), 1.0 / sm.dist));
  out->steps_used = 1u;
  return out->contact ? FBS_TRACE_CONTACT : FBS_TRACE_OK;
}

/* ------------------------------------------------------------------------- */
/* Context                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct fbs_trace_pair {
  unsigned target_id, zone_id;
} fbs_trace_pair;

typedef struct fbs_trace_slot {
  int in_use;
  unsigned weapon_id;
  unsigned attack_id;
  unsigned window_count;
  fbs_trace_window *windows;
  int has_previous;
  int has_current;
  float previous_time, current_time;
  fbs_trace_weapon previous, current;
  unsigned struck_count;
  fbs_trace_pair *struck;
} fbs_trace_slot;

struct fbs_trace_context {
  fbs_trace_config config;
  fbs_trace_allocator allocator;
  void *block;
  size_t block_bytes;
  fbs_trace_slot *slots;
};

/* Largest count accepted for any per-context array. */
#define FBS_MAX_COUNT (1u << 20)

static void *fbs_default_alloc(void *user, size_t bytes) {
  (void)user;
  return malloc(bytes);
}

static void fbs_default_free(void *user, void *ptr) {
  (void)user;
  free(ptr);
}

static int size_mul(size_t a, size_t b, size_t *out) {
  if (a != 0u && b > SIZE_MAX / a) return 0;
  *out = a * b;
  return 1;
}

static int size_add(size_t a, size_t b, size_t *out) {
  if (b > SIZE_MAX - a) return 0;
  *out = a + b;
  return 1;
}

static int size_align(size_t n, size_t *out) {
  if (n > SIZE_MAX - (FBS_ALIGN - 1u)) return 0;
  *out = (n + (FBS_ALIGN - 1u)) & ~(FBS_ALIGN - 1u);
  return 1;
}

static fbs_trace_slot *slot_find(const fbs_trace_context *ctx, unsigned weapon_id) {
  unsigned i;
  for (i = 0; i < ctx->config.max_weapons; ++i) {
    if (ctx->slots[i].in_use && ctx->slots[i].weapon_id == weapon_id) return &ctx->slots[i];
  }
  return NULL;
}

static void slot_clear_samples(fbs_trace_slot *sl) {
  sl->has_previous = 0;
  sl->has_current = 0;
  sl->previous_time = 0.0f;
  sl->current_time = 0.0f;
  memset(&sl->previous, 0, sizeof sl->previous);
  memset(&sl->current, 0, sizeof sl->current);
}

static int slot_active(const fbs_trace_slot *sl) {
  unsigned i;
  if (!sl->has_previous) return 0;
  if (sl->window_count == 0u) return 1;
  for (i = 0; i < sl->window_count; ++i) {
    if (sl->windows[i].begin <= sl->current_time && sl->windows[i].end >= sl->previous_time)
      return 1;
  }
  return 0;
}

static int slot_is_struck(const fbs_trace_slot *sl, unsigned target_id, unsigned zone_id) {
  unsigned i;
  for (i = 0; i < sl->struck_count; ++i) {
    if (sl->struck[i].target_id != target_id) continue;
    if (sl->struck[i].zone_id == FBS_TRACE_ZONE_ANY || sl->struck[i].zone_id == zone_id) return 1;
  }
  return 0;
}

static fbs_trace_status slot_mark(const fbs_trace_context *ctx, fbs_trace_slot *sl,
                                  unsigned target_id, unsigned zone_id) {
  if (slot_is_struck(sl, target_id, zone_id)) return FBS_TRACE_OK;
  if (sl->struck_count >= ctx->config.max_struck) return FBS_TRACE_E_DEDUP;
  sl->struck[sl->struck_count].target_id = target_id;
  sl->struck[sl->struck_count].zone_id = zone_id;
  sl->struck_count++;
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_create(const fbs_trace_config *cfg, const fbs_trace_allocator *alloc,
                                  fbs_trace_context **out) {
  fbs_trace_config c = fbs_trace_config_default();
  fbs_trace_allocator a;
  size_t need_ctx = 0, need_slots = 0, need_windows = 0, need_struck = 0, tmp = 0, total = 0;
  unsigned char *raw;
  unsigned char *p;
  fbs_trace_context *ctx;
  unsigned i;

  if (out == NULL) return FBS_TRACE_E_INVALID;
  if (cfg != NULL) c = *cfg;
  if (c.max_weapons < 1u || c.max_weapons > FBS_MAX_COUNT) return FBS_TRACE_E_INVALID;
  if (c.max_windows < 1u || c.max_windows > FBS_MAX_COUNT) return FBS_TRACE_E_INVALID;
  if (c.max_struck < 1u || c.max_struck > FBS_MAX_COUNT) return FBS_TRACE_E_INVALID;
  if (c.dedup != (int)FBS_TRACE_DEDUP_NONE && c.dedup != (int)FBS_TRACE_DEDUP_TARGET &&
      c.dedup != (int)FBS_TRACE_DEDUP_TARGET_ZONE && c.dedup != (int)FBS_TRACE_DEDUP_MANUAL)
    return FBS_TRACE_E_INVALID;
  if (!policy_ok(&c.policy)) return FBS_TRACE_E_INVALID;

  a.alloc = fbs_default_alloc;
  a.free = fbs_default_free;
  a.user = NULL;
  if (alloc != NULL) {
    if (alloc->alloc == NULL || alloc->free == NULL) return FBS_TRACE_E_INVALID;
    a = *alloc;
  }

  /* One block: context, slots, all window arrays, all dedup tables. Every
   * product is overflow-checked because size_t is 32 bits under wasm32. */
  if (!size_align(sizeof(struct fbs_trace_context), &need_ctx)) return FBS_TRACE_E_INVALID;
  if (!size_mul((size_t)c.max_weapons, sizeof(fbs_trace_slot), &tmp)) return FBS_TRACE_E_INVALID;
  if (!size_align(tmp, &need_slots)) return FBS_TRACE_E_INVALID;
  if (!size_mul((size_t)c.max_weapons, (size_t)c.max_windows, &tmp)) return FBS_TRACE_E_INVALID;
  if (!size_mul(tmp, sizeof(fbs_trace_window), &tmp)) return FBS_TRACE_E_INVALID;
  if (!size_align(tmp, &need_windows)) return FBS_TRACE_E_INVALID;
  if (!size_mul((size_t)c.max_weapons, (size_t)c.max_struck, &tmp)) return FBS_TRACE_E_INVALID;
  if (!size_mul(tmp, sizeof(fbs_trace_pair), &tmp)) return FBS_TRACE_E_INVALID;
  if (!size_align(tmp, &need_struck)) return FBS_TRACE_E_INVALID;

  total = need_ctx;
  if (!size_add(total, need_slots, &total)) return FBS_TRACE_E_INVALID;
  if (!size_add(total, need_windows, &total)) return FBS_TRACE_E_INVALID;
  if (!size_add(total, need_struck, &total)) return FBS_TRACE_E_INVALID;
  if (!size_add(total, FBS_ALIGN - 1u, &total)) return FBS_TRACE_E_INVALID;

  raw = (unsigned char *)a.alloc(a.user, total);
  if (raw == NULL) return FBS_TRACE_E_MEMORY;
  memset(raw, 0, total);

  p = raw + (size_t)((FBS_ALIGN - ((uintptr_t)raw & (FBS_ALIGN - 1u))) & (FBS_ALIGN - 1u));
  ctx = (fbs_trace_context *)(void *)p;
  p += need_ctx;
  ctx->config = c;
  ctx->allocator = a;
  ctx->block = raw;
  ctx->block_bytes = total;
  ctx->slots = (fbs_trace_slot *)(void *)p;
  p += need_slots;
  for (i = 0; i < c.max_weapons; ++i) {
    ctx->slots[i].windows = (fbs_trace_window *)(void *)(p + (size_t)i * (size_t)c.max_windows *
                                                                 sizeof(fbs_trace_window));
  }
  p += need_windows;
  for (i = 0; i < c.max_weapons; ++i) {
    ctx->slots[i].struck = (fbs_trace_pair *)(void *)(p + (size_t)i * (size_t)c.max_struck *
                                                              sizeof(fbs_trace_pair));
  }

  *out = ctx;
  return FBS_TRACE_OK;
}

void fbs_trace_destroy(fbs_trace_context *ctx) {
  if (ctx == NULL) return;
  ctx->allocator.free(ctx->allocator.user, ctx->block);
}

void fbs_trace_reset(fbs_trace_context *ctx) {
  unsigned i;
  if (ctx == NULL) return;
  for (i = 0; i < ctx->config.max_weapons; ++i) {
    fbs_trace_slot *sl = &ctx->slots[i];
    sl->in_use = 0;
    sl->weapon_id = 0u;
    sl->attack_id = 0u;
    sl->window_count = 0u;
    sl->struck_count = 0u;
    slot_clear_samples(sl);
  }
}

size_t fbs_trace_memory(const fbs_trace_context *ctx) {
  return ctx != NULL ? ctx->block_bytes : (size_t)0;
}

fbs_trace_status fbs_trace_begin(fbs_trace_context *ctx, unsigned weapon_id, unsigned attack_id,
                                 const fbs_trace_window *windows, size_t window_count) {
  fbs_trace_slot *sl;
  size_t i;
  unsigned n;

  if (ctx == NULL) return FBS_TRACE_E_INVALID;
  if (window_count > 0u && windows == NULL) return FBS_TRACE_E_INVALID;
  if (window_count > (size_t)ctx->config.max_windows) return FBS_TRACE_E_INVALID;
  for (i = 0; i < window_count; ++i) {
    if (!fbs_finite(windows[i].begin) || !fbs_finite(windows[i].end)) return FBS_TRACE_E_INVALID;
    if (windows[i].begin > windows[i].end) return FBS_TRACE_E_INVALID;
  }

  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) {
    unsigned s;
    for (s = 0; s < ctx->config.max_weapons; ++s) {
      if (!ctx->slots[s].in_use) {
        sl = &ctx->slots[s];
        break;
      }
    }
    if (sl == NULL) return FBS_TRACE_E_CAPACITY;
  }

  n = (unsigned)window_count;
  sl->in_use = 1;
  sl->weapon_id = weapon_id;
  sl->attack_id = attack_id;
  sl->window_count = n;
  for (i = 0; i < window_count; ++i) sl->windows[i] = windows[i];
  sl->struck_count = 0u;
  slot_clear_samples(sl);
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_update(fbs_trace_context *ctx, unsigned weapon_id, unsigned attack_id,
                                  float time, const fbs_trace_weapon *pose) {
  fbs_trace_slot *sl;
  if (ctx == NULL || !weapon_ok(pose) || !fbs_finite(time)) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_STATE; /* update without begin */
  if (sl->attack_id != attack_id) return FBS_TRACE_E_STATE;
  if (sl->has_current && time < sl->current_time) return FBS_TRACE_E_STATE;

  if (sl->has_current) {
    sl->previous = sl->current;
    sl->previous_time = sl->current_time;
    sl->has_previous = 1;
  }
  sl->current = *pose;
  sl->current_time = time;
  sl->has_current = 1;
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_break(fbs_trace_context *ctx, unsigned weapon_id) {
  fbs_trace_slot *sl;
  if (ctx == NULL) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;
  /* Drop both samples: the next update is a fresh first sample, so no interval
   * can span the teleport and the host may rewind its attack clock (hit-stop)
   * without losing the session's windows or dedup memory. */
  slot_clear_samples(sl);
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_end(fbs_trace_context *ctx, unsigned weapon_id) {
  fbs_trace_slot *sl;
  if (ctx == NULL) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;
  sl->in_use = 0;
  sl->window_count = 0u;
  sl->struck_count = 0u;
  slot_clear_samples(sl);
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_interval_get(const fbs_trace_context *ctx, unsigned weapon_id,
                                        fbs_trace_interval *out) {
  const fbs_trace_slot *sl;
  if (ctx == NULL || out == NULL) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;
  memset(out, 0, sizeof *out);
  out->attack_id = sl->attack_id;
  out->has_previous = sl->has_previous;
  out->active = slot_active(sl);
  out->previous_time = sl->previous_time;
  out->current_time = sl->current_time;
  out->previous = sl->previous;
  out->current = sl->current;
  return FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_substeps(const fbs_trace_context *ctx, unsigned weapon_id,
                                    fbs_trace_weapon *out, size_t cap, size_t *count) {
  const fbs_trace_slot *sl;
  fbs_dvec3 a0, b0, a1, b1;
  double travel, dsteps;
  unsigned steps, i;
  size_t n;

  if (ctx == NULL || count == NULL) return FBS_TRACE_E_INVALID;
  if (out == NULL && cap > 0u) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;
  if (!slot_active(sl)) {
    *count = 0u;
    return FBS_TRACE_OK;
  }

  if (ctx->config.policy.mode == (int)FBS_TRACE_MODE_ADVANCE) {
    /* Exact contact times come from fbs_trace_test; hosts only need the ends.
     * A short cap still gets as many entries as fit, per the E_FULL rule, so
     * cap == 1 receives the previous pose rather than nothing. */
    *count = 2u;
    if (cap > 0u) out[0] = sl->previous;
    if (cap > 1u) out[1] = sl->current;
    return cap < 2u ? FBS_TRACE_E_FULL : FBS_TRACE_OK;
  }

  a0 = dv_of(sl->previous.a);
  b0 = dv_of(sl->previous.b);
  a1 = dv_of(sl->current.a);
  b1 = dv_of(sl->current.b);
  travel = d_max(dv_len(dv_sub(a1, a0)), dv_len(dv_sub(b1, b0)));
  dsteps = ceil(travel / (double)ctx->config.policy.spacing);
  if (!(dsteps >= 1.0)) dsteps = 1.0;
  if (dsteps + 1.0 > (double)ctx->config.policy.max_steps) {
    *count = 0u;
    return FBS_TRACE_E_CAPACITY;
  }
  steps = (unsigned)dsteps;
  n = (size_t)steps + 1u;
  *count = n;
  for (i = 0; i <= steps && (size_t)i < cap; ++i) {
    double t = (double)i / (double)steps;
    out[i].a = dv_to(dv_lerp(a0, a1, t));
    out[i].b = dv_to(dv_lerp(b0, b1, t));
    out[i].radius = (float)((double)sl->previous.radius +
                            t * ((double)sl->current.radius - (double)sl->previous.radius));
  }
  return n > cap ? FBS_TRACE_E_FULL : FBS_TRACE_OK;
}

/* Builds the sweep for one target against the slot's interval. */
static void target_sweep(const fbs_trace_slot *sl, const fbs_trace_target *tg, fbs_sweep *s) {
  const fbs_trace_capsule *prev = tg->has_previous ? &tg->previous : &tg->current;
  fbs_dvec3 c0 = dv_of(prev->a);
  fbs_dvec3 d0 = tg->kind == (int)FBS_TRACE_TARGET_SPHERE ? c0 : dv_of(prev->b);
  fbs_dvec3 c1 = dv_of(tg->current.a);
  fbs_dvec3 d1 = tg->kind == (int)FBS_TRACE_TARGET_SPHERE ? c1 : dv_of(tg->current.b);
  sweep_setup(s, &sl->previous, &sl->current, c0, d0, c1, d1, (double)prev->radius);
}

static int target_ok(const fbs_trace_target *tg) {
  if (tg->zone_id == FBS_TRACE_ZONE_ANY) return 0;
  if (tg->kind != (int)FBS_TRACE_TARGET_SPHERE && tg->kind != (int)FBS_TRACE_TARGET_CAPSULE)
    return 0;
  if (!capsule_ok(&tg->current)) return 0;
  if (tg->has_previous && !capsule_ok(&tg->previous)) return 0;
  return 1;
}

fbs_trace_status fbs_trace_test(fbs_trace_context *ctx, unsigned weapon_id,
                                const fbs_trace_target *targets, size_t target_count,
                                fbs_trace_hit *hits, size_t cap, size_t *count) {
  fbs_trace_slot *sl;
  size_t i, written = 0u, found = 0u;
  int dedup_full = 0;

  if (ctx == NULL || count == NULL) return FBS_TRACE_E_INVALID;
  if (target_count > 0u && targets == NULL) return FBS_TRACE_E_INVALID;
  if (hits == NULL && cap > 0u) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;

  /* Validate every target before anything is written or marked. */
  for (i = 0; i < target_count; ++i) {
    if (!target_ok(&targets[i])) {
      *count = i;
      return FBS_TRACE_E_INVALID;
    }
  }

  if (!slot_active(sl)) {
    *count = 0u;
    return FBS_TRACE_OK;
  }

  /* Pass 1: capacity probe only, so FBS_TRACE_E_CAPACITY leaves `hits`
   * untouched without needing scratch storage for the contacts. */
  for (i = 0; i < target_count; ++i) {
    fbs_sweep s;
    fbs_trace_status st;
    if (slot_is_struck(sl, targets[i].target_id, targets[i].zone_id)) continue;
    target_sweep(sl, &targets[i], &s);
    st = sweep_run(&s, &ctx->config.policy, NULL);
    if (st < 0) {
      *count = i;
      return st;
    }
  }

  /* Pass 2: collect contacts, insertion-sorted by time, ties keeping input
   * order, keeping only the earliest `cap`. */
  for (i = 0; i < target_count; ++i) {
    fbs_sweep s;
    fbs_trace_contact contact;
    size_t pos, j;
    if (slot_is_struck(sl, targets[i].target_id, targets[i].zone_id)) continue;
    target_sweep(sl, &targets[i], &s);
    if (sweep_run(&s, &ctx->config.policy, &contact) != FBS_TRACE_CONTACT) continue;
    found++;
    pos = written;
    while (pos > 0u && hits[pos - 1u].contact.time > contact.time) pos--;
    if (pos >= cap) continue; /* later than everything already kept */
    if (written < cap) written++;
    for (j = written - 1u; j > pos; --j) hits[j] = hits[j - 1u];
    hits[pos].target_id = targets[i].target_id;
    hits[pos].zone_id = targets[i].zone_id;
    hits[pos].contact = contact;
  }
  *count = written;

  /* Auto-mark only what was written, after the hits are in the caller's array. */
  if (ctx->config.dedup == (int)FBS_TRACE_DEDUP_TARGET ||
      ctx->config.dedup == (int)FBS_TRACE_DEDUP_TARGET_ZONE) {
    for (i = 0; i < written; ++i) {
      unsigned zone = ctx->config.dedup == (int)FBS_TRACE_DEDUP_TARGET ? FBS_TRACE_ZONE_ANY
                                                                       : hits[i].zone_id;
      if (slot_mark(ctx, sl, hits[i].target_id, zone) == FBS_TRACE_E_DEDUP) dedup_full = 1;
    }
  }

  if (found > cap) return FBS_TRACE_E_FULL;
  return dedup_full ? FBS_TRACE_E_DEDUP : FBS_TRACE_OK;
}

fbs_trace_status fbs_trace_mark(fbs_trace_context *ctx, unsigned weapon_id, unsigned target_id,
                                unsigned zone_id) {
  fbs_trace_slot *sl;
  if (ctx == NULL) return FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return FBS_TRACE_E_NOT_FOUND;
  return slot_mark(ctx, sl, target_id, zone_id);
}

int fbs_trace_is_struck(const fbs_trace_context *ctx, unsigned weapon_id, unsigned target_id,
                        unsigned zone_id) {
  const fbs_trace_slot *sl;
  if (ctx == NULL) return (int)FBS_TRACE_E_INVALID;
  sl = slot_find(ctx, weapon_id);
  if (sl == NULL) return (int)FBS_TRACE_E_NOT_FOUND;
  return slot_is_struck(sl, target_id, zone_id);
}
