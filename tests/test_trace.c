/*
 * tests/test_trace.c — tests for fbs/trace.h 0.1.0, no framework.
 *
 * Exit code is the number of failed checks; the run prints "N checks passed".
 * One function per numbered item of docs/lanes/trace-impl-brief.md. Every
 * expected number is derived analytically in the comment above its test.
 *
 * Fixture mode:
 *   ./fbs_test_trace --write-fixtures tests/fixtures/trace/sweeps.json
 * The default run reloads that file (path from $FBS_TRACE_FIXTURES, else
 * fixtures/trace/sweeps.json relative to the working directory, which is
 * tests/ under ctest) and re-checks every case within 1e-6.
 */
#include "fbs/trace.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------------------- */
/* Check plumbing (no global state: the tally is threaded through)           */
/* ------------------------------------------------------------------------- */

typedef struct tally {
  int checks;
  int failures;
} tally;

static void record(tally *t, int ok, const char *what, const char *file, int line) {
  t->checks++;
  if (!ok) {
    t->failures++;
    printf("FAIL %s:%d: %s\n", file, line, what);
  }
}

static void record_near(tally *t, double got, double want, double eps, const char *what,
                        const char *file, int line) {
  /* Written so that a NaN fails both comparisons instead of passing one. */
  int ok = (got - want <= eps) && (want - got <= eps);
  t->checks++;
  if (!ok) {
    t->failures++;
    printf("FAIL %s:%d: %s: got %.17g want %.17g (eps %g)\n", file, line, what, got, want, eps);
  }
}

#define CHECK(T, cond) record((T), (cond) ? 1 : 0, #cond, __FILE__, __LINE__)
#define NEAR(T, got, want, eps) \
  record_near((T), (double)(got), (double)(want), (double)(eps), #got, __FILE__, __LINE__)
#define NEAR3(T, v, X, Y, Z, eps)     \
  do {                                \
    NEAR((T), (v).x, (X), (eps));     \
    NEAR((T), (v).y, (Y), (eps));     \
    NEAR((T), (v).z, (Z), (eps));     \
  } while (0)

/* ------------------------------------------------------------------------- */
/* Small constructors                                                        */
/* ------------------------------------------------------------------------- */

static fbs_vec3 v3(float x, float y, float z) {
  fbs_vec3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

static fbs_trace_weapon wep(fbs_vec3 a, fbs_vec3 b, float radius) {
  fbs_trace_weapon w;
  w.a = a;
  w.b = b;
  w.radius = radius;
  return w;
}

static fbs_trace_capsule cap(fbs_vec3 a, fbs_vec3 b, float radius) {
  fbs_trace_capsule c;
  c.a = a;
  c.b = b;
  c.radius = radius;
  return c;
}

static fbs_trace_sphere sph(fbs_vec3 center, float radius) {
  fbs_trace_sphere s;
  s.center = center;
  s.radius = radius;
  return s;
}

static fbs_trace_target tgt(unsigned target_id, unsigned zone_id, int kind, fbs_trace_capsule cur) {
  fbs_trace_target t;
  memset(&t, 0, sizeof t);
  t.target_id = target_id;
  t.zone_id = zone_id;
  t.kind = kind;
  t.has_previous = 0;
  t.current = cur;
  t.previous = cur;
  return t;
}

static fbs_trace_policy pol(int mode, float spacing, float tolerance, unsigned max_steps) {
  fbs_trace_policy p;
  p.mode = mode;
  p.spacing = spacing;
  p.tolerance = tolerance;
  p.max_steps = max_steps;
  return p;
}

static int all_finite(const fbs_trace_contact *c) {
  const float f[15] = {c->time,           c->weapon_param,    c->target_param, c->distance,
                       c->point.x,        c->point.y,         c->point.z,      c->target_point.x,
                       c->target_point.y, c->target_point.z,  c->normal.x,     c->normal.y,
                       c->normal.z,       c->motion.x,        c->motion.y};
  int i;
  for (i = 0; i < 15; ++i) {
    double d = (double)f[i];
    if (!isfinite(d)) return 0;
  }
  return isfinite((double)c->motion.z) && isfinite((double)c->relative_motion.x) &&
         isfinite((double)c->relative_motion.y) && isfinite((double)c->relative_motion.z);
}

/* Weapon pose at normalized time t, per the documented per-endpoint lerp. */
static fbs_vec3 lerp3(fbs_vec3 p0, fbs_vec3 p1, double t) {
  fbs_vec3 r;
  r.x = (float)((double)p0.x + t * ((double)p1.x - (double)p0.x));
  r.y = (float)((double)p0.y + t * ((double)p1.y - (double)p0.y));
  r.z = (float)((double)p0.z + t * ((double)p1.z - (double)p0.z));
  return r;
}

/* ------------------------------------------------------------------------- */
/* 1. Thin target, fast sweep                                                */
/* ------------------------------------------------------------------------- */

/* Weapon is the segment (x,-0.5,0)..(x,0.5,0) translating x: -1.01 -> +1.01,
 * so the closest point to a sphere on the x axis is (x,0,0) at k = 0.5 and the
 * separation is |x|. Sphere: centre at the origin, radius 0.019, R = 0.019.
 *
 * SAMPLED, spacing 0.04: travel = max(2.02, 2.02) + 0 = 2.02, so
 * steps = ceil(2.02/0.04) = 51 and the samples sit at x = -1.01 + 2.02 i/51,
 * i.e. 2.02/51 = 39.6 mm apart. The pair that brackets the origin is
 * i = 25 (x = -0.0198039) and i = 26 (x = +0.0198039): the 19 mm sphere fits
 * between two samples with 0.8 mm of margin on each side, so a 4 cm schedule
 * MISSES it entirely. steps_used = steps + 1 = 52.
 * (2.02 is used instead of a round 2.00 on purpose: 2.00/0.04 is an exact
 * integer, and float(0.04) = 0.0399999991 makes ceil() land one step higher
 * than the double 0.04 of the TypeScript original. See docs/lanes/trace-impl.md.)
 *
 * ADVANCE, tolerance 1e-4: L = 2.02. Iteration 0 at t = 0: dist = 1.01,
 * sep = 1.01 - 0.019 = 0.991, t <- 0.991/2.02 = 0.4905940594. Iteration 1:
 * x = -1.01 + 2.02(0.4905940594) = -0.019, dist = 0.019 = R, sep = 0 <= 1e-4
 * -> contact. So time is the analytic first-touch time (1.01 - 0.019)/2.02
 * = 0.49059405940594054, distance = R, steps_used = 2, point = (-0.019,0,0),
 * normal = (-1,0,0), motion = relative_motion = (2.02,0,0). */
static void test_thin_target_fast_sweep(tally *t) {
  fbs_trace_weapon w0 = wep(v3(-1.01f, -0.5f, 0.0f), v3(-1.01f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon w1 = wep(v3(1.01f, -0.5f, 0.0f), v3(1.01f, 0.5f, 0.0f), 0.0f);
  fbs_trace_sphere s = sph(v3(0.0f, 0.0f, 0.0f), 0.019f);
  fbs_trace_policy sampled = fbs_trace_policy_compat_4cm();
  fbs_trace_policy advance = fbs_trace_policy_default();
  fbs_trace_contact c;

  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &sampled, &c) == FBS_TRACE_OK);
  CHECK(t, c.contact == 0);
  CHECK(t, c.steps_used == 52u);

  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &advance, &c) == FBS_TRACE_CONTACT);
  CHECK(t, c.contact == 1);
  CHECK(t, c.steps_used == 2u);
  NEAR(t, c.time, 0.49059405940594054, 1e-4);
  NEAR(t, c.time, 0.49059405940594054, 1e-6); /* advancement lands on it exactly */
  NEAR(t, c.distance, 0.019, advance.tolerance);
  NEAR(t, c.weapon_param, 0.5, 1e-6);
  NEAR(t, c.target_param, 0.0, 1e-6);
  NEAR3(t, c.point, -0.019, 0.0, 0.0, 1e-6);
  NEAR3(t, c.target_point, 0.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.normal, -1.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.motion, 2.02, 0.0, 0.0, 1e-6);
  NEAR3(t, c.relative_motion, 2.02, 0.0, 0.0, 1e-6);
  CHECK(t, all_finite(&c));
}

/* ------------------------------------------------------------------------- */
/* 2. Endpoint and middle contact                                            */
/* ------------------------------------------------------------------------- */

/* Weapon is the segment (0,y,0)..(1,y,0) translating y: 0 -> 0.98 (travel 0.98,
 * steps = ceil(0.98/0.04) = 25, samples y = 0.98 i/25 = 0.0392 i).
 *  - tip case: sphere at (1, 0.5, 0), radius 0.12. The closest weapon point is
 *    the tip (k = 1) and dist = |y - 0.5|; contact needs y >= 0.38. Sample
 *    i = 9 has y = 0.3528 (dist 0.1472 > 0.12), i = 10 has y = 0.392
 *    (dist 0.108 <= 0.12), so time = 10/25 = 0.40, distance = 0.108,
 *    steps_used = 11, point = (1, 0.392, 0), motion = (0, 0.98, 0).
 *  - middle case: sphere at (0.5, 0.5, 0) -> k = 0.5, same time/distance.
 *  - capsule case: axis (1,0.5,-0.3)..(1,0.5,0.3), radius 0.12. The weapon
 *    lives in z = 0, so the closest axis point is its midpoint, u = 0.5, and
 *    the numbers match the tip case.
 * (0.98 rather than 1.00 so that travel/spacing is 24.5, safely away from the
 * integer where float(0.04) and double 0.04 disagree on ceil().) */
static void test_endpoint_and_middle_contact(tally *t) {
  fbs_trace_weapon w0 = wep(v3(0.0f, 0.0f, 0.0f), v3(1.0f, 0.0f, 0.0f), 0.0f);
  fbs_trace_weapon w1 = wep(v3(0.0f, 0.98f, 0.0f), v3(1.0f, 0.98f, 0.0f), 0.0f);
  fbs_trace_policy p = fbs_trace_policy_compat_4cm();
  fbs_trace_sphere tip = sph(v3(1.0f, 0.5f, 0.0f), 0.12f);
  fbs_trace_sphere mid = sph(v3(0.5f, 0.5f, 0.0f), 0.12f);
  fbs_trace_capsule axis = cap(v3(1.0f, 0.5f, -0.3f), v3(1.0f, 0.5f, 0.3f), 0.12f);
  fbs_trace_contact c;
  fbs_vec3 a, b, recomputed;
  fbs_trace_capsule as_sphere;

  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &tip, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.40, 1e-6);
  NEAR(t, c.weapon_param, 1.0, 1e-6);
  NEAR(t, c.target_param, 0.0, 1e-6);
  NEAR(t, c.distance, 0.108, 1e-6);
  CHECK(t, c.steps_used == 11u);
  NEAR3(t, c.point, 1.0, 0.392, 0.0, 1e-6);
  NEAR3(t, c.target_point, 1.0, 0.5, 0.0, 1e-6);
  NEAR3(t, c.normal, 0.0, -1.0, 0.0, 1e-6);
  NEAR3(t, c.motion, 0.0, 0.98, 0.0, 1e-6);

  /* point must lie on the interpolated segment at `time`. */
  a = lerp3(w0.a, w1.a, (double)c.time);
  b = lerp3(w0.b, w1.b, (double)c.time);
  recomputed = lerp3(a, b, (double)c.weapon_param);
  NEAR3(t, c.point, (double)recomputed.x, (double)recomputed.y, (double)recomputed.z, 1e-6);

  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &mid, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.40, 1e-6);
  NEAR(t, c.weapon_param, 0.5, 1e-6);
  NEAR(t, c.distance, 0.108, 1e-6);
  CHECK(t, c.steps_used == 11u);
  NEAR3(t, c.point, 0.5, 0.392, 0.0, 1e-6);
  a = lerp3(w0.a, w1.a, (double)c.time);
  b = lerp3(w0.b, w1.b, (double)c.time);
  recomputed = lerp3(a, b, (double)c.weapon_param);
  NEAR3(t, c.point, (double)recomputed.x, (double)recomputed.y, (double)recomputed.z, 1e-6);

  CHECK(t, fbs_trace_sweep_capsule(&w0, &w1, &axis, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.40, 1e-6);
  NEAR(t, c.weapon_param, 1.0, 1e-6);
  NEAR(t, c.target_param, 0.5, 1e-6);
  NEAR(t, c.distance, 0.108, 1e-6);
  NEAR3(t, c.point, 1.0, 0.392, 0.0, 1e-6);
  NEAR3(t, c.target_point, 1.0, 0.5, 0.0, 1e-6);

  /* fbs_trace_closest: static pose at the contact time reproduces the same
   * closest points, fills even when clear, and flags contact itself. */
  {
    fbs_trace_weapon at_contact = wep(a, b, 0.0f);
    fbs_trace_contact k;
    CHECK(t, fbs_trace_closest(&at_contact, &axis, &k) == FBS_TRACE_CONTACT);
    NEAR(t, k.time, 0.0, 1e-9);
    NEAR(t, k.weapon_param, 1.0, 1e-6);
    NEAR(t, k.target_param, 0.5, 1e-6);
    NEAR(t, k.distance, 0.108, 1e-6);
    NEAR3(t, k.motion, 0.0, 0.0, 0.0, 1e-9);
    NEAR3(t, k.relative_motion, 0.0, 0.0, 0.0, 1e-9);
    CHECK(t, k.steps_used == 1u);
  }
  /* Contact only at the inclusive final sample t = 1: the weapon
   * (0,0,0)..(1,0,0) translates 0.9 m along +z against a sphere at
   * (0.5,0,1.0) of radius 0.1005. travel = 0.9 so steps = ceil(0.9/0.04) = 23
   * and the samples sit at z = 0.9 i/23. The last-but-one sample (i = 22,
   * z = 0.8608696) is 0.1391304 away, outside R; only i = 23 (z = 0.9,
   * distance exactly 0.1) is inside it. So time = 1, weapon_param = 0.5,
   * target_param = 0, distance = 0.1, steps_used = 24. Dropping the inclusive
   * upper endpoint from the sample loop turns this into a miss. */
  {
    fbs_trace_weapon f0 = wep(v3(0.0f, 0.0f, 0.0f), v3(1.0f, 0.0f, 0.0f), 0.0f);
    fbs_trace_weapon f1 = wep(v3(0.0f, 0.0f, 0.9f), v3(1.0f, 0.0f, 0.9f), 0.0f);
    fbs_trace_sphere last = sph(v3(0.5f, 0.0f, 1.0f), 0.1005f);
    fbs_trace_contact k;
    CHECK(t, fbs_trace_sweep_sphere(&f0, &f1, &last, NULL, &p, &k) == FBS_TRACE_CONTACT);
    NEAR(t, k.time, 1.0, 0.0);
    NEAR(t, k.weapon_param, 0.5, 1e-6);
    NEAR(t, k.target_param, 0.0, 1e-6);
    NEAR(t, k.distance, 0.1, 1e-6);
    CHECK(t, k.steps_used == 24u);
    NEAR3(t, k.point, 0.5, 0.0, 0.9, 1e-6);
    NEAR3(t, k.target_point, 0.5, 0.0, 1.0, 1e-6);
    NEAR3(t, k.normal, 0.0, 0.0, -1.0, 1e-6);
    NEAR3(t, k.motion, 0.0, 0.0, 0.9, 1e-6);
    /* and the sample just before it really is outside R */
    {
      fbs_trace_weapon m0 =
          wep(lerp3(f0.a, f1.a, 22.0 / 23.0), lerp3(f0.b, f1.b, 22.0 / 23.0), 0.0f);
      fbs_trace_capsule as_point = cap(last.center, last.center, last.radius);
      fbs_trace_contact q;
      CHECK(t, fbs_trace_closest(&m0, &as_point, &q) == FBS_TRACE_OK);
      NEAR(t, q.distance, 0.13913043478260871, 1e-6);
    }
  }

  as_sphere = cap(v3(1.0f, 5.0f, 0.0f), v3(1.0f, 5.0f, 0.0f), 0.1f);
  {
    fbs_trace_contact k;
    CHECK(t, fbs_trace_closest(&w0, &as_sphere, &k) == FBS_TRACE_OK);
    CHECK(t, k.contact == 0);
    NEAR(t, k.distance, 5.0, 1e-6); /* fields are filled even with no contact */
    NEAR(t, k.weapon_param, 1.0, 1e-6);
    NEAR3(t, k.normal, 0.0, -1.0, 0.0, 1e-6);
  }
}

/* ------------------------------------------------------------------------- */
/* 3. Rotating weapon                                                        */
/* ------------------------------------------------------------------------- */

/* Base fixed at the origin, tip of a blade of length L = 0.98 rotating 60
 * degrees: b0 = L(1,0,0), b1 = L(1/2, sqrt(3)/2, 0). The sphere sits on the
 * chord midpoint C = (b0+b1)/2, radius 0.05.
 *
 * Writing b(t) = L b_unit(t), C.b(t) = C.b0 + t C.(b1-b0)
 * = L^2 (1 + 1/2)/2 + t (|b1|^2 - |b0|^2)/2 = 0.75 L^2 for every t, and
 * |b(t)|^2 = L^2 ((1 - t/2)^2 + (3/4) t^2) = L^2 (t^2 - t + 1). Hence
 *   k(t) = 0.75 / (t^2 - t + 1)                       (scale independent)
 *   dist(t)^2 = |C|^2 - k(t) 0.75 L^2 = 0.75 L^2 (t - 1/2)^2 / (t^2 - t + 1).
 * travel = |b1 - b0| = 2 L sin 30 = L = 0.98, so steps = ceil(0.98/0.04) = 25
 * and the samples are t = 0.04 i. dist <= 0.05 first holds at
 * |t - 1/2| <= 0.05110919, i.e. t >= 0.44889081, so the first contacting
 * sample is t = 0.48 (t = 0.44 gives 0.05866). At t = 0.48:
 * t^2 - t + 1 = 0.7504, k = 0.75/0.7504 = 0.9994669509594883,
 * dist = 0.98 sqrt(0.75) 0.02 / sqrt(0.7504) = 0.019594775134261694,
 * steps_used = 13, motion = (b1 - b0) k because a1 - a0 = 0, and
 * point = k b(0.48) = (0.744403, 0.407161, 0). */
static void test_rotating_weapon(tally *t) {
  const float root3_2 = 0.8660254037844386f; /* sin 60 */
  const float len = 0.98f;
  fbs_trace_weapon w0 = wep(v3(0.0f, 0.0f, 0.0f), v3(len, 0.0f, 0.0f), 0.0f);
  fbs_trace_weapon w1 = wep(v3(0.0f, 0.0f, 0.0f), v3(len * 0.5f, len * root3_2, 0.0f), 0.0f);
  fbs_trace_sphere s = sph(v3((len + len * 0.5f) * 0.5f, (len * root3_2) * 0.5f, 0.0f), 0.05f);
  fbs_trace_policy p = fbs_trace_policy_compat_4cm();
  fbs_trace_contact c;
  double k;

  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.48, 1e-6);
  CHECK(t, c.steps_used == 13u);
  NEAR(t, c.weapon_param, 0.9994669509594883, 1e-6);
  NEAR(t, c.distance, 0.019594775134261694, 1e-6);
  CHECK(t, c.weapon_param >= 0.0f && c.weapon_param <= 1.0f);

  /* motion == (b1 - b0) k, and point == a(t) + k (b(t) - a(t)). */
  k = (double)c.weapon_param;
  NEAR(t, c.motion.x, ((double)w1.b.x - (double)w0.b.x) * k, 1e-6);
  NEAR(t, c.motion.y, ((double)w1.b.y - (double)w0.b.y) * k, 1e-6);
  NEAR(t, c.motion.z, 0.0, 1e-9);
  NEAR3(t, c.relative_motion, (double)c.motion.x, (double)c.motion.y, 0.0, 1e-9);
  {
    fbs_vec3 a = lerp3(w0.a, w1.a, (double)c.time);
    fbs_vec3 b = lerp3(w0.b, w1.b, (double)c.time);
    fbs_vec3 on_segment = lerp3(a, b, k);
    NEAR3(t, c.point, (double)on_segment.x, (double)on_segment.y, (double)on_segment.z, 1e-6);
    NEAR(t, c.point.x, 0.74440299891955042, 1e-5);
    NEAR(t, c.point.y, 0.40716118678251140, 1e-5);
  }
  CHECK(t, all_finite(&c));
}

/* ------------------------------------------------------------------------- */
/* 4. Zero-length weapon (fist), held pose                                   */
/* ------------------------------------------------------------------------- */

/* Fist at the origin, w0 == w1 so travel = 0 and steps = max(1, 0) = 1
 * (samples t = 0 and t = 1). Capsule (0.1,-0.5,0)..(0.1,0.5,0) radius 0.2:
 * the degenerate weapon takes Ericson's a <= eps branch, k = 0, and the
 * closest axis point is the midpoint (u = 0.5) at (0.1,0,0). dist = 0.1 <= 0.2
 * -> contact at t = 0 with steps_used 1 in both modes.
 * Moving the capsule to x = 1 gives dist = 1 > 0.2: SAMPLED evaluates both
 * samples (steps_used 2), ADVANCE sees L = 0 after one evaluation and stops
 * (steps_used 1). */
static void test_zero_length_weapon_held_pose(tally *t) {
  fbs_trace_weapon fist = wep(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.0f);
  fbs_trace_capsule near = cap(v3(0.1f, -0.5f, 0.0f), v3(0.1f, 0.5f, 0.0f), 0.2f);
  fbs_trace_capsule far = cap(v3(1.0f, -0.5f, 0.0f), v3(1.0f, 0.5f, 0.0f), 0.2f);
  fbs_trace_policy sampled = fbs_trace_policy_compat_4cm();
  fbs_trace_policy advance = fbs_trace_policy_default();
  fbs_trace_contact c;

  CHECK(t, fbs_trace_sweep_capsule(&fist, &fist, &near, NULL, &sampled, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.0, 1e-9);
  NEAR(t, c.weapon_param, 0.0, 1e-9);
  NEAR(t, c.target_param, 0.5, 1e-6);
  NEAR(t, c.distance, 0.1, 1e-6);
  NEAR3(t, c.target_point, 0.1, 0.0, 0.0, 1e-6);
  NEAR3(t, c.normal, -1.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.motion, 0.0, 0.0, 0.0, 1e-9);
  CHECK(t, c.steps_used == 1u);

  CHECK(t, fbs_trace_sweep_capsule(&fist, &fist, &near, NULL, &advance, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.0, 1e-9);
  NEAR(t, c.distance, 0.1, 1e-6);
  CHECK(t, c.steps_used == 1u);

  CHECK(t, fbs_trace_sweep_capsule(&fist, &fist, &far, NULL, &sampled, &c) == FBS_TRACE_OK);
  CHECK(t, c.contact == 0);
  CHECK(t, c.steps_used == 2u);
  CHECK(t, fbs_trace_sweep_capsule(&fist, &fist, &far, NULL, &advance, &c) == FBS_TRACE_OK);
  CHECK(t, c.contact == 0);
  CHECK(t, c.steps_used == 1u);
}

/* ------------------------------------------------------------------------- */
/* 5. Target translation                                                     */
/* ------------------------------------------------------------------------- */

/* Stationary weapon (0,-0.5,0)..(0,0.5,0); sphere radius 0.1 moving
 * (-1.01,0,0) -> (1.01,0,0). travel = 0 + 2.02 = 2.02, so steps = 51 and the
 * sphere centre is at x = -1.01 + 2.02 i/51. The closest weapon point is the
 * origin (k = 0.5) and dist = |x|; contact needs x >= -0.1, i.e.
 * t >= 0.91/2.02 = 0.45049505, so the first sample is i = 23:
 * t = 23/51 = 0.45098039, x = -0.09901961, distance 0.09901961 (i = 22 gives
 * 0.13862745 > 0.1), steps_used = 24. The weapon does not move, so motion = 0
 * and relative_motion = -(target motion) = (-2.02,0,0). */
static void test_target_translation(tally *t) {
  fbs_trace_weapon w = wep(v3(0.0f, -0.5f, 0.0f), v3(0.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_sphere s0 = sph(v3(-1.01f, 0.0f, 0.0f), 0.1f);
  fbs_trace_sphere s1 = sph(v3(1.01f, 0.0f, 0.0f), 0.1f);
  fbs_trace_policy p = fbs_trace_policy_compat_4cm();
  fbs_trace_contact c;

  CHECK(t, fbs_trace_sweep_sphere(&w, &w, &s0, &s1, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.time, 0.45098039215686275, 1e-6);
  CHECK(t, c.steps_used == 24u);
  NEAR(t, c.weapon_param, 0.5, 1e-6);
  NEAR(t, c.distance, 0.099019607843137, 1e-6);
  NEAR3(t, c.point, 0.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.target_point, -0.099019607843137, 0.0, 0.0, 1e-6);
  NEAR3(t, c.normal, 1.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.motion, 0.0, 0.0, 0.0, 1e-9);
  NEAR3(t, c.relative_motion, -2.02, 0.0, 0.0, 1e-6);

  /* A stationary weapon against a stationary sphere far away must not move
   * the sampler at all: travel 0 -> two samples, no contact. */
  {
    fbs_trace_sphere away = sph(v3(9.0f, 0.0f, 0.0f), 0.1f);
    CHECK(t, fbs_trace_sweep_sphere(&w, &w, &away, NULL, &p, &c) == FBS_TRACE_OK);
    CHECK(t, c.steps_used == 2u);
  }
}

/* ------------------------------------------------------------------------- */
/* 6. Parallel and collinear axes                                            */
/* ------------------------------------------------------------------------- */

/* Parallel: weapon (0,0,0)..(1,0,0) held, capsule (0,0.15,0)..(1,0.15,0)
 * radius 0.2. denom = a e - b^2 = 1 - 1 = 0 <= 1e-12, so Ericson's parallel
 * branch picks s = 0, then u = (b s + f)/e = f = d2.(p1-p2) = 0 and both
 * parameters stay in range: k = 0, u = 0, dist = 0.15 <= 0.2 -> contact.
 * Collinear disjoint: capsule (2,0,0)..(3,0,0) radius 0.2. s = 0 gives
 * u = -2 < 0, so the fix-up sets u = 0 and k = clamp(-c/a) = 1: the pair is
 * (1,0,0)/(2,0,0) at distance 1 > 0.2 -> clear, no NaN.
 * Collinear overlapping: capsule (0.5,0,0)..(1.5,0,0) radius 0.1 gives u = 0,
 * k = 0.5 and dist = 0 exactly, which must produce a zero normal, not NaN. */
static void test_parallel_and_collinear(tally *t) {
  fbs_trace_weapon w = wep(v3(0.0f, 0.0f, 0.0f), v3(1.0f, 0.0f, 0.0f), 0.0f);
  fbs_trace_policy p = fbs_trace_policy_compat_4cm();
  fbs_trace_capsule parallel = cap(v3(0.0f, 0.15f, 0.0f), v3(1.0f, 0.15f, 0.0f), 0.2f);
  fbs_trace_capsule collinear_far = cap(v3(2.0f, 0.0f, 0.0f), v3(3.0f, 0.0f, 0.0f), 0.2f);
  fbs_trace_capsule collinear_over = cap(v3(0.5f, 0.0f, 0.0f), v3(1.5f, 0.0f, 0.0f), 0.1f);
  fbs_trace_contact c;

  CHECK(t, fbs_trace_sweep_capsule(&w, &w, &parallel, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.distance, 0.15, 1e-6);
  NEAR(t, c.weapon_param, 0.0, 1e-9);
  NEAR(t, c.target_param, 0.0, 1e-9);
  NEAR3(t, c.normal, 0.0, -1.0, 0.0, 1e-6);
  CHECK(t, all_finite(&c));
  CHECK(t, c.weapon_param >= 0.0f && c.weapon_param <= 1.0f);
  CHECK(t, c.target_param >= 0.0f && c.target_param <= 1.0f);

  CHECK(t, fbs_trace_sweep_capsule(&w, &w, &collinear_far, NULL, &p, &c) == FBS_TRACE_OK);
  CHECK(t, c.contact == 0);
  CHECK(t, c.steps_used == 2u);
  CHECK(t, fbs_trace_closest(&w, &collinear_far, &c) == FBS_TRACE_OK);
  NEAR(t, c.distance, 1.0, 1e-6);
  NEAR(t, c.weapon_param, 1.0, 1e-9);
  NEAR(t, c.target_param, 0.0, 1e-9);
  NEAR3(t, c.point, 1.0, 0.0, 0.0, 1e-6);
  NEAR3(t, c.target_point, 2.0, 0.0, 0.0, 1e-6);
  CHECK(t, all_finite(&c));

  CHECK(t, fbs_trace_sweep_capsule(&w, &w, &collinear_over, NULL, &p, &c) == FBS_TRACE_CONTACT);
  NEAR(t, c.distance, 0.0, 1e-9);
  NEAR(t, c.weapon_param, 0.5, 1e-6);
  NEAR3(t, c.normal, 0.0, 0.0, 0.0, 0.0); /* coincident points -> zero normal */
  CHECK(t, all_finite(&c));

  /* Degenerate everything: point weapon vs point capsule at the same place. */
  {
    fbs_trace_weapon pt = wep(v3(2.0f, 2.0f, 2.0f), v3(2.0f, 2.0f, 2.0f), 0.0f);
    fbs_trace_capsule pc = cap(v3(2.0f, 2.0f, 2.0f), v3(2.0f, 2.0f, 2.0f), 0.0f);
    CHECK(t, fbs_trace_sweep_capsule(&pt, &pt, &pc, NULL, &p, &c) == FBS_TRACE_CONTACT);
    NEAR(t, c.distance, 0.0, 0.0);
    CHECK(t, all_finite(&c));
  }
  /* Coordinates at 1e6: squared lengths near 1e12 must not overflow. */
  {
    fbs_trace_weapon b0 = wep(v3(999950.0f, -0.5f, 0.0f), v3(999950.0f, 0.5f, 0.0f), 0.0f);
    fbs_trace_weapon b1 = wep(v3(1000050.0f, -0.5f, 0.0f), v3(1000050.0f, 0.5f, 0.0f), 0.0f);
    fbs_trace_sphere s = sph(v3(1000000.0f, 0.0f, 0.0f), 1.2f);
    fbs_trace_policy big = pol((int)FBS_TRACE_MODE_SAMPLED, 1.0f, 1e-4f, 1024u);
    CHECK(t, fbs_trace_sweep_sphere(&b0, &b1, &s, NULL, &big, &c) == FBS_TRACE_CONTACT);
    NEAR(t, c.time, 0.49, 1e-6);
    NEAR(t, c.distance, 1.0, 1e-3);
    CHECK(t, all_finite(&c));
  }
}

/* ------------------------------------------------------------------------- */
/* 7. Frame-rate schedules (30 / 60 / 144 Hz)                                */
/* ------------------------------------------------------------------------- */

/* The weapon segment (x,-0.5,0)..(x,0.5,0) advances at 3 m/s from x = -1;
 * the sphere at the origin has radius 0.1, so the analytic first touch is at
 * x = -0.1, i.e. tau = 0.9/3 = 0.3 s. Every schedule feeds fbs_trace_update
 * once per frame and calls fbs_trace_test; TARGET_ZONE dedup must leave
 * exactly one hit for the whole swing. Global contact time is
 * previous_time + time * (current_time - previous_time). SAMPLED lands on a
 * 4 cm sample so its error is bounded by one frame; ADVANCE converges to the
 * true crossing (measured: exactly 0.300000 s at all three rates). */
static void run_schedule(tally *t, int hz, int mode) {
  fbs_trace_config cfg = fbs_trace_config_default();
  fbs_trace_context *ctx = NULL;
  double dt = 1.0 / (double)hz;
  int frame;
  int hits_total = 0;
  double global = -1.0;

  cfg.policy = mode == (int)FBS_TRACE_MODE_SAMPLED ? fbs_trace_policy_compat_4cm()
                                                   : fbs_trace_policy_default();
  cfg.dedup = (int)FBS_TRACE_DEDUP_TARGET_ZONE;
  CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
  if (ctx == NULL) return;
  CHECK(t, fbs_trace_begin(ctx, 1u, 42u, NULL, 0) == FBS_TRACE_OK);

  for (frame = 0; (double)frame * dt <= 0.7000001; ++frame) {
    double tau = (double)frame * dt;
    float x = (float)(-1.0 + 3.0 * tau);
    fbs_trace_weapon pose = wep(v3(x, -0.5f, 0.0f), v3(x, 0.5f, 0.0f), 0.0f);
    fbs_trace_target target = tgt(7u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                                  cap(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.1f));
    fbs_trace_hit hits[4];
    size_t n = 0;
    fbs_trace_interval iv;

    CHECK(t, fbs_trace_update(ctx, 1u, 42u, (float)tau, &pose) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_test(ctx, 1u, &target, 1u, hits, 4u, &n) == FBS_TRACE_OK);
    if (n > 0u) {
      CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
      global = (double)iv.previous_time +
               (double)hits[0].contact.time * ((double)iv.current_time - (double)iv.previous_time);
      hits_total += (int)n;
    }
  }
  CHECK(t, hits_total == 1);
  if (mode == (int)FBS_TRACE_MODE_SAMPLED) {
    NEAR(t, global, 0.3, dt);
  } else {
    NEAR(t, global, 0.3, 1e-3);
  }
  CHECK(t, fbs_trace_is_struck(ctx, 1u, 7u, 0u) == 1);
  fbs_trace_destroy(ctx);
}

static void test_frame_rate_schedules(tally *t) {
  run_schedule(t, 30, (int)FBS_TRACE_MODE_SAMPLED);
  run_schedule(t, 60, (int)FBS_TRACE_MODE_SAMPLED);
  run_schedule(t, 144, (int)FBS_TRACE_MODE_SAMPLED);
  run_schedule(t, 30, (int)FBS_TRACE_MODE_ADVANCE);
  run_schedule(t, 60, (int)FBS_TRACE_MODE_ADVANCE);
  run_schedule(t, 144, (int)FBS_TRACE_MODE_ADVANCE);
}

/* ------------------------------------------------------------------------- */
/* 8. Lifecycle                                                              */
/* ------------------------------------------------------------------------- */

/* No numeric derivation needed; every transition of the documented state
 * machine is exercised. The checkpoint case is the one that matters for the
 * ported bug "cross-attack stitching": the weapon is left with a previous
 * sample at x = -1, then begin() restarts the session and a single update at
 * x = +1 must NOT sweep the -1 -> +1 span through the sphere at the origin. */
static void test_lifecycle(tally *t) {
  fbs_trace_config cfg = fbs_trace_config_default();
  fbs_trace_context *ctx = NULL;
  fbs_trace_weapon pose_a = wep(v3(-1.0f, -0.5f, 0.0f), v3(-1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon pose_b = wep(v3(-0.58f, -0.5f, 0.0f), v3(-0.58f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon pose_c = wep(v3(1.0f, -0.5f, 0.0f), v3(1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_target sphere = tgt(1u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                                cap(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.1f));
  fbs_trace_hit hits[8];
  fbs_trace_interval iv;
  fbs_trace_weapon poses[64];
  size_t n = 0;

  cfg.max_weapons = 2u;
  cfg.policy = fbs_trace_policy_compat_4cm();
  CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
  if (ctx == NULL) return;

  /* update / break / end / interval before begin */
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.0f, &pose_a) == FBS_TRACE_E_STATE);
  CHECK(t, fbs_trace_break(ctx, 1u) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_end(ctx, 1u) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 64u, &n) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_test(ctx, 1u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_mark(ctx, 1u, 1u, 0u) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_is_struck(ctx, 1u, 1u, 0u) == FBS_TRACE_E_NOT_FOUND);

  CHECK(t, fbs_trace_begin(ctx, 1u, 7u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.attack_id == 7u);
  CHECK(t, iv.has_previous == 0 && iv.active == 0);

  /* attack id mismatch */
  CHECK(t, fbs_trace_update(ctx, 1u, 8u, 0.0f, &pose_a) == FBS_TRACE_E_STATE);

  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.0f, &pose_a) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 0); /* first sample only sets current */
  CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 64u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 0u); /* inactive interval yields no poses */

  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.1f, &pose_b) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 1 && iv.active == 1);
  NEAR(t, iv.previous_time, 0.0, 0.0);
  NEAR(t, iv.current_time, 0.1, 1e-6);
  NEAR(t, iv.previous.a.x, -1.0, 0.0);
  NEAR(t, iv.current.a.x, -0.58, 1e-6);

  /* substeps: weapon travel 0.42 -> steps = ceil(0.42/0.04) = 11 -> 12 poses
   * at x = -1 + 0.42 i/11, the first exactly the previous pose and the last
   * exactly the current pose. */
  CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 64u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 12u);
  NEAR(t, poses[0].a.x, -1.0, 1e-6);
  NEAR(t, poses[11].a.x, -0.58, 1e-6);
  NEAR(t, poses[5].a.x, -0.80909090909090908, 1e-6);
  NEAR(t, poses[0].a.y, -0.5, 1e-6);
  NEAR(t, poses[11].b.y, 0.5, 1e-6);
  CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 4u, &n) == FBS_TRACE_E_FULL);
  CHECK(t, n == 12u); /* count is what was needed, 4 poses were written */
  NEAR(t, poses[3].a.x, -0.88545454545454544, 1e-6);

  /* time must not go backwards; equal times are legal (held pose) */
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.05f, &pose_b) == FBS_TRACE_E_STATE);
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.1f, &pose_b) == FBS_TRACE_OK);

  /* break discards BOTH samples: the next update is a fresh first sample and
   * may carry any finite time (a host rewinding its swing clock on hit-stop),
   * while the session, its windows and its dedup memory survive. The session
   * stands at time 0.1 here, so 0.02 would be rejected without the break. */
  CHECK(t, fbs_trace_mark(ctx, 1u, 99u, 3u) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_break(ctx, 1u) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 0 && iv.active == 0);
  CHECK(t, iv.attack_id == 7u);
  CHECK(t, fbs_trace_is_struck(ctx, 1u, 99u, 3u) == 1); /* dedup memory survives */
  CHECK(t, fbs_trace_test(ctx, 1u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 0u);
  CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 64u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 0u);
  /* time goes backwards across the break and is accepted */
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.02f, &pose_a) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 0); /* a fresh first sample, not a re-arm */
  NEAR(t, iv.current_time, 0.02, 1e-6);
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.06f, &pose_c) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 1); /* the interval spans the two post-break samples */
  NEAR(t, iv.previous_time, 0.02, 1e-6);
  NEAR(t, iv.current_time, 0.06, 1e-6);
  NEAR(t, iv.previous.a.x, -1.0, 1e-6); /* pose_a, pushed after the break */
  NEAR(t, iv.current.a.x, 1.0, 1e-6);   /* pose_c; the pre-break pose_b is gone */
  /* monotonicity applies again inside the post-break run */
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.05f, &pose_c) == FBS_TRACE_E_STATE);

  /* end frees the slot */
  CHECK(t, fbs_trace_end(ctx, 1u) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_end(ctx, 1u) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_update(ctx, 1u, 7u, 0.3f, &pose_b) == FBS_TRACE_E_STATE);

  /* checkpoint: a restarted session must not stitch onto the old sample */
  CHECK(t, fbs_trace_begin(ctx, 1u, 11u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 11u, 0.0f, &pose_a) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 11u, 0.1f, &pose_c) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_test(ctx, 1u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 1u); /* -1 -> +1 really does cross the sphere */
  CHECK(t, fbs_trace_begin(ctx, 1u, 12u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.has_previous == 0);
  CHECK(t, fbs_trace_is_struck(ctx, 1u, 1u, 0u) == 0); /* begin cleared dedup */
  CHECK(t, fbs_trace_update(ctx, 1u, 12u, 0.0f, &pose_c) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_test(ctx, 1u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 0u); /* no interval: the old -1 pose is gone */

  /* reset clears every session */
  CHECK(t, fbs_trace_begin(ctx, 2u, 1u, NULL, 0) == FBS_TRACE_OK);
  fbs_trace_reset(ctx);
  CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_interval_get(ctx, 2u, &iv) == FBS_TRACE_E_NOT_FOUND);

  /* windows: [0.375, 0.5] inclusive overlap with [previous_time, current_time] */
  {
    fbs_trace_window win;
    win.begin = 0.375f;
    win.end = 0.5f;
    CHECK(t, fbs_trace_begin(ctx, 3u, 1u, &win, 1u) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.20f, &pose_a) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.30f, &pose_b) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 3u, &iv) == FBS_TRACE_OK);
    CHECK(t, iv.has_previous == 1 && iv.active == 0); /* entirely before */
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.375f, &pose_b) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 3u, &iv) == FBS_TRACE_OK);
    CHECK(t, iv.active == 1); /* touches the window edge */
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.50f, &pose_b) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 3u, &iv) == FBS_TRACE_OK);
    CHECK(t, iv.active == 1);
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.60f, &pose_b) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 3u, &iv) == FBS_TRACE_OK);
    CHECK(t, iv.active == 1); /* [0.50, 0.60] still touches the window end */
    CHECK(t, fbs_trace_update(ctx, 3u, 1u, 0.90f, &pose_b) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 3u, &iv) == FBS_TRACE_OK);
    CHECK(t, iv.active == 0); /* [0.60, 0.90] is entirely after */
  }
  fbs_trace_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* 9. Dual weapons                                                           */
/* ------------------------------------------------------------------------- */

/* Two sessions with different attack ids. Weapon 1 sweeps x: -1 -> +1 through
 * a sphere at the origin (hit); weapon 2 sweeps the same span 5 m above it
 * (clear). Samples, dedup memory and lifetime must be per weapon. */
static void test_dual_weapons(tally *t) {
  fbs_trace_config cfg = fbs_trace_config_default();
  fbs_trace_context *ctx = NULL;
  fbs_trace_weapon lo0 = wep(v3(-1.0f, -0.5f, 0.0f), v3(-1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon lo1 = wep(v3(1.0f, -0.5f, 0.0f), v3(1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon hi0 = wep(v3(-1.0f, 4.5f, 0.0f), v3(-1.0f, 5.5f, 0.0f), 0.0f);
  fbs_trace_weapon hi1 = wep(v3(1.0f, 4.5f, 0.0f), v3(1.0f, 5.5f, 0.0f), 0.0f);
  fbs_trace_target sphere = tgt(1u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                                cap(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.1f));
  fbs_trace_hit hits[8];
  fbs_trace_interval iv;
  size_t n = 0;

  cfg.max_weapons = 2u;
  cfg.policy = fbs_trace_policy_compat_4cm();
  CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
  if (ctx == NULL) return;

  CHECK(t, fbs_trace_begin(ctx, 10u, 100u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_begin(ctx, 20u, 200u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 10u, 100u, 0.0f, &lo0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 20u, 200u, 0.0f, &hi0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 10u, 100u, 0.1f, &lo1) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 20u, 200u, 0.1f, &hi1) == FBS_TRACE_OK);

  /* ids do not cross */
  CHECK(t, fbs_trace_update(ctx, 10u, 200u, 0.2f, &lo1) == FBS_TRACE_E_STATE);
  CHECK(t, fbs_trace_interval_get(ctx, 10u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.attack_id == 100u);
  NEAR(t, iv.previous.a.y, -0.5, 1e-6);
  CHECK(t, fbs_trace_interval_get(ctx, 20u, &iv) == FBS_TRACE_OK);
  CHECK(t, iv.attack_id == 200u);
  NEAR(t, iv.previous.a.y, 4.5, 1e-6);

  CHECK(t, fbs_trace_test(ctx, 10u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 1u);
  CHECK(t, fbs_trace_test(ctx, 20u, &sphere, 1u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 0u);

  /* dedup memory is per weapon */
  CHECK(t, fbs_trace_is_struck(ctx, 10u, 1u, 0u) == 1);
  CHECK(t, fbs_trace_is_struck(ctx, 20u, 1u, 0u) == 0);
  CHECK(t, fbs_trace_mark(ctx, 20u, 55u, 1u) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_is_struck(ctx, 10u, 55u, 1u) == 0);
  CHECK(t, fbs_trace_is_struck(ctx, 20u, 55u, 1u) == 1);

  /* ending one leaves the other alone */
  CHECK(t, fbs_trace_end(ctx, 10u) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_interval_get(ctx, 10u, &iv) == FBS_TRACE_E_NOT_FOUND);
  CHECK(t, fbs_trace_interval_get(ctx, 20u, &iv) == FBS_TRACE_OK);
  fbs_trace_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* 10. Multiple targets: ordering, tie stability, truncation                 */
/* ------------------------------------------------------------------------- */

/* Weapon segment (x,-1,0)..(x,1,0) sweeps x: -1.02 -> 0 (travel 1.02,
 * steps = ceil(1.02/0.04) = 26, samples x = -1.02 + 1.02 i/26). Four sphere
 * zones of radius 0.05, contact when x >= centre_x - 0.05:
 *   zone 10 at (-0.10, 0,   0): x >= -0.15 -> t >= 0.852941 -> i=23, t=0.884615
 *   zone 11 at (-0.30, 0.5, 0): x >= -0.35 -> t >= 0.656863 -> i=18, t=0.692308
 *   zone 12 at (-0.30,-0.5, 0): same time (k = 0.25 instead of 0.75)
 *   zone 13 at (-0.20, 0,   0): x >= -0.25 -> t >= 0.754902 -> i=20, t=0.769231
 * Distances at those samples: 0.0176923 (z10), 0.0138461 (z11/z12),
 * 0.0353846 (z13). Fed in the order 10, 11, 12, 13, the result must be
 * 11, 12, 13, 10: ascending time with the tie keeping input order. cap = 1
 * keeps only zone 11 and returns FBS_TRACE_E_FULL with *count = 1. */
static void test_multiple_targets(tally *t) {
  fbs_trace_config cfg = fbs_trace_config_default();
  fbs_trace_context *ctx = NULL;
  fbs_trace_weapon p0 = wep(v3(-1.02f, -1.0f, 0.0f), v3(-1.02f, 1.0f, 0.0f), 0.0f);
  fbs_trace_weapon p1 = wep(v3(0.0f, -1.0f, 0.0f), v3(0.0f, 1.0f, 0.0f), 0.0f);
  fbs_trace_target targets[4];
  fbs_trace_hit hits[8];
  size_t n = 0;

  targets[0] = tgt(1u, 10u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(-0.1f, 0.0f, 0.0f), v3(-0.1f, 0.0f, 0.0f), 0.05f));
  targets[1] = tgt(1u, 11u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(-0.3f, 0.5f, 0.0f), v3(-0.3f, 0.5f, 0.0f), 0.05f));
  targets[2] = tgt(1u, 12u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(-0.3f, -0.5f, 0.0f), v3(-0.3f, -0.5f, 0.0f), 0.05f));
  targets[3] = tgt(1u, 13u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(-0.2f, 0.0f, 0.0f), v3(-0.2f, 0.0f, 0.0f), 0.05f));

  cfg.policy = fbs_trace_policy_compat_4cm();
  CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
  if (ctx == NULL) return;

  CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &p0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 1u, 1.0f, &p1) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_test(ctx, 1u, targets, 4u, hits, 8u, &n) == FBS_TRACE_OK);
  CHECK(t, n == 4u);
  if (n == 4u) {
    CHECK(t, hits[0].zone_id == 11u && hits[1].zone_id == 12u && hits[2].zone_id == 13u &&
                 hits[3].zone_id == 10u);
    NEAR(t, hits[0].contact.time, 0.69230769230769229, 1e-6);
    NEAR(t, hits[1].contact.time, 0.69230769230769229, 1e-6);
    NEAR(t, hits[2].contact.time, 0.76923076923076927, 1e-6);
    NEAR(t, hits[3].contact.time, 0.88461538461538458, 1e-6);
    CHECK(t, hits[0].contact.time <= hits[1].contact.time);
    CHECK(t, hits[1].contact.time <= hits[2].contact.time);
    CHECK(t, hits[2].contact.time <= hits[3].contact.time);
    NEAR(t, hits[0].contact.weapon_param, 0.75, 1e-6);
    NEAR(t, hits[1].contact.weapon_param, 0.25, 1e-6);
    NEAR(t, hits[0].contact.distance, 0.013846136, 1e-6);
    NEAR(t, hits[2].contact.distance, 0.035384608, 1e-6);
    NEAR(t, hits[3].contact.distance, 0.017692304, 1e-6);
    CHECK(t, hits[0].target_id == 1u);
  }

  /* cap = 1: only the earliest hit is written and only it is marked. */
  CHECK(t, fbs_trace_begin(ctx, 2u, 1u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 2u, 1u, 0.0f, &p0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 2u, 1u, 1.0f, &p1) == FBS_TRACE_OK);
  memset(hits, 0, sizeof hits);
  CHECK(t, fbs_trace_test(ctx, 2u, targets, 4u, hits, 1u, &n) == FBS_TRACE_E_FULL);
  CHECK(t, n == 1u);
  CHECK(t, hits[0].zone_id == 11u);
  NEAR(t, hits[0].contact.time, 0.69230769230769229, 1e-6);
  CHECK(t, hits[1].zone_id == 0u); /* nothing past cap was touched */
  CHECK(t, fbs_trace_is_struck(ctx, 2u, 1u, 11u) == 1);
  CHECK(t, fbs_trace_is_struck(ctx, 2u, 1u, 12u) == 0); /* did not fit -> not marked */
  fbs_trace_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* 11. Duplicate policy                                                      */
/* ------------------------------------------------------------------------- */

/* Weapon segment (x,-1,0)..(x,1,0). Interval A sweeps x: -0.5 -> 0, interval B
 * sweeps x: 0 -> 0.5 (each travel 0.5, steps = ceil(0.5/0.04) = 13).
 * Zones of target 5, all spheres:
 *   zone 0 at (0, 0.5, 0) r 0.3 and zone 1 at (0,-0.5,0) r 0.3 are both within
 *   reach in BOTH intervals (dist = |x|, contact when |x| <= 0.3).
 *   zone 2 at (0.4, 0, 0) r 0.05 is out of reach in A (dist >= 0.4) and hit in
 *   B at t = 10/13.
 * Expected hit counts (A, B): TARGET_ZONE (2, 1) — zone 2 is new; TARGET
 * (2, 0) — the whole target is silenced; NONE (2, 3) — nothing is remembered;
 * MANUAL (2, 3), then (2, 1) after marking zones 0 and 1 by hand. */
static void run_dedup(tally *t, int dedup, int *a_hits, int *b_hits) {
  fbs_trace_config cfg = fbs_trace_config_default();
  fbs_trace_context *ctx = NULL;
  fbs_trace_weapon pa = wep(v3(-0.5f, -1.0f, 0.0f), v3(-0.5f, 1.0f, 0.0f), 0.0f);
  fbs_trace_weapon pb = wep(v3(0.0f, -1.0f, 0.0f), v3(0.0f, 1.0f, 0.0f), 0.0f);
  fbs_trace_weapon pc = wep(v3(0.5f, -1.0f, 0.0f), v3(0.5f, 1.0f, 0.0f), 0.0f);
  fbs_trace_target targets[3];
  fbs_trace_hit hits[8];
  size_t n = 0;

  targets[0] = tgt(5u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(0.0f, 0.5f, 0.0f), v3(0.0f, 0.5f, 0.0f), 0.3f));
  targets[1] = tgt(5u, 1u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(0.0f, -0.5f, 0.0f), v3(0.0f, -0.5f, 0.0f), 0.3f));
  targets[2] = tgt(5u, 2u, (int)FBS_TRACE_TARGET_SPHERE,
                   cap(v3(0.4f, 0.0f, 0.0f), v3(0.4f, 0.0f, 0.0f), 0.05f));

  *a_hits = -1;
  *b_hits = -1;
  cfg.dedup = dedup;
  cfg.policy = fbs_trace_policy_compat_4cm();
  CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
  if (ctx == NULL) return;
  CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &pa) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.1f, &pb) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_test(ctx, 1u, targets, 3u, hits, 8u, &n) == FBS_TRACE_OK);
  *a_hits = (int)n;

  if (dedup == (int)FBS_TRACE_DEDUP_MANUAL) {
    /* Re-running the same interval must report the same hits until marked. */
    CHECK(t, fbs_trace_test(ctx, 1u, targets, 3u, hits, 8u, &n) == FBS_TRACE_OK);
    CHECK(t, (int)n == *a_hits);
    CHECK(t, fbs_trace_mark(ctx, 1u, 5u, 0u) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_mark(ctx, 1u, 5u, 0u) == FBS_TRACE_OK); /* idempotent */
    CHECK(t, fbs_trace_mark(ctx, 1u, 5u, 1u) == FBS_TRACE_OK);
  }
  CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.2f, &pc) == FBS_TRACE_OK);
  CHECK(t, fbs_trace_test(ctx, 1u, targets, 3u, hits, 8u, &n) == FBS_TRACE_OK);
  *b_hits = (int)n;

  if (dedup == (int)FBS_TRACE_DEDUP_TARGET_ZONE) {
    CHECK(t, n == 1u && hits[0].zone_id == 2u);
    CHECK(t, fbs_trace_is_struck(ctx, 1u, 5u, 0u) == 1);
    CHECK(t, fbs_trace_is_struck(ctx, 1u, 5u, 9u) == 0); /* other zones free */
    /* ZONE_ANY silences everything, including zones never reported. */
    CHECK(t, fbs_trace_mark(ctx, 1u, 5u, FBS_TRACE_ZONE_ANY) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_is_struck(ctx, 1u, 5u, 9u) == 1);
    CHECK(t, fbs_trace_is_struck(ctx, 1u, 6u, 0u) == 0); /* other targets free */
    CHECK(t, fbs_trace_test(ctx, 1u, targets, 3u, hits, 8u, &n) == FBS_TRACE_OK);
    CHECK(t, n == 0u);
  }
  if (dedup == (int)FBS_TRACE_DEDUP_NONE) {
    CHECK(t, fbs_trace_is_struck(ctx, 1u, 5u, 0u) == 0); /* nothing auto-marked */
  }
  fbs_trace_destroy(ctx);
}

static void test_duplicates(tally *t) {
  int a = 0, b = 0;
  run_dedup(t, (int)FBS_TRACE_DEDUP_TARGET_ZONE, &a, &b);
  CHECK(t, a == 2 && b == 1);
  run_dedup(t, (int)FBS_TRACE_DEDUP_TARGET, &a, &b);
  CHECK(t, a == 2 && b == 0);
  run_dedup(t, (int)FBS_TRACE_DEDUP_NONE, &a, &b);
  CHECK(t, a == 2 && b == 3);
  run_dedup(t, (int)FBS_TRACE_DEDUP_MANUAL, &a, &b);
  CHECK(t, a == 2 && b == 1);
}

/* ------------------------------------------------------------------------- */
/* 12. Malformed input                                                       */
/* ------------------------------------------------------------------------- */

/* Every rejected call must leave its output byte-for-byte untouched, so the
 * output is pre-filled with 0xA5 and compared afterwards. */
static int untouched(const void *p, size_t n, unsigned char byte) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; ++i) {
    if (b[i] != byte) return 0;
  }
  return 1;
}

static void test_malformed_input(tally *t) {
  fbs_trace_weapon w0 = wep(v3(-1.0f, -0.5f, 0.0f), v3(-1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_weapon w1 = wep(v3(1.0f, -0.5f, 0.0f), v3(1.0f, 0.5f, 0.0f), 0.0f);
  fbs_trace_sphere s = sph(v3(0.0f, 0.0f, 0.0f), 0.1f);
  fbs_trace_capsule cc = cap(v3(0.0f, -0.2f, 0.0f), v3(0.0f, 0.2f, 0.0f), 0.1f);
  fbs_trace_policy good = fbs_trace_policy_compat_4cm();
  fbs_trace_contact c;
  double nan_value = 0.0;
  float nan_f, inf_f;

  /* Build a NaN and an infinity without relying on NAN/INFINITY macros. */
  {
    double zero = 0.0;
    double one = 1.0;
    nan_value = zero / zero;
    inf_f = (float)(one / zero);
    nan_f = (float)nan_value;
  }
  CHECK(t, !isfinite((double)nan_f) && !isfinite((double)inf_f));

#define BAD_SWEEP(call)                                  \
  do {                                                   \
    memset(&c, 0xA5, sizeof c);                          \
    CHECK(t, (call) == FBS_TRACE_E_INVALID);             \
    CHECK(t, untouched(&c, sizeof c, 0xA5));             \
  } while (0)

  /* NULL pointers */
  BAD_SWEEP(fbs_trace_sweep_sphere(NULL, &w1, &s, NULL, &good, &c));
  BAD_SWEEP(fbs_trace_sweep_sphere(&w0, NULL, &s, NULL, &good, &c));
  BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, NULL, NULL, &good, &c));
  BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, NULL, &c));
  BAD_SWEEP(fbs_trace_sweep_capsule(&w0, &w1, NULL, NULL, &good, &c));
  BAD_SWEEP(fbs_trace_closest(NULL, &cc, &c));
  BAD_SWEEP(fbs_trace_closest(&w0, NULL, &c));
  CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &good, NULL) == FBS_TRACE_E_INVALID);
  CHECK(t, fbs_trace_sweep_capsule(&w0, &w1, &cc, NULL, &good, NULL) == FBS_TRACE_E_INVALID);
  CHECK(t, fbs_trace_closest(&w0, &cc, NULL) == FBS_TRACE_E_INVALID);

  /* non-finite coordinates in each shape slot */
  {
    fbs_trace_weapon bad = w0;
    bad.a.x = nan_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&bad, &w1, &s, NULL, &good, &c));
    bad = w1;
    bad.b.z = inf_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &bad, &s, NULL, &good, &c));
    bad = w0;
    bad.radius = nan_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&bad, &w1, &s, NULL, &good, &c));
    bad = w0;
    bad.radius = -0.001f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&bad, &w1, &s, NULL, &good, &c));
  }
  {
    fbs_trace_sphere bad = s;
    bad.center.y = inf_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &bad, NULL, &good, &c));
    bad = s;
    bad.radius = -1.0f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &bad, NULL, &good, &c));
    bad = s;
    bad.center.x = nan_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, &bad, &good, &c)); /* the moving pose */
  }
  {
    fbs_trace_capsule bad = cc;
    bad.b.x = nan_f;
    BAD_SWEEP(fbs_trace_sweep_capsule(&w0, &w1, &bad, NULL, &good, &c));
    bad = cc;
    bad.radius = -0.5f;
    BAD_SWEEP(fbs_trace_sweep_capsule(&w0, &w1, &bad, NULL, &good, &c));
    bad = cc;
    bad.a.z = inf_f;
    BAD_SWEEP(fbs_trace_sweep_capsule(&w0, &w1, &cc, &bad, &good, &c));
    BAD_SWEEP(fbs_trace_closest(&w0, &bad, &c));
  }

  /* bad policies */
  {
    fbs_trace_policy p = good;
    p.spacing = 0.0f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.spacing = -0.04f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.spacing = nan_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.max_steps = 1u;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.max_steps = 0u;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.mode = 7;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = good;
    p.mode = -1;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = fbs_trace_policy_default();
    p.tolerance = 0.0f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    p = fbs_trace_policy_default();
    p.tolerance = nan_f;
    BAD_SWEEP(fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c));
    /* SAMPLED ignores tolerance and ADVANCE ignores spacing */
    p = good;
    p.tolerance = -5.0f;
    CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c) == FBS_TRACE_CONTACT);
    p = fbs_trace_policy_default();
    p.spacing = -5.0f;
    CHECK(t, fbs_trace_sweep_sphere(&w0, &w1, &s, NULL, &p, &c) == FBS_TRACE_CONTACT);
  }
#undef BAD_SWEEP

  /* context-level validation */
  {
    fbs_trace_context *ctx = NULL;
    fbs_trace_context *sentinel;
    fbs_trace_config cfg = fbs_trace_config_default();
    fbs_trace_window win[2];
    fbs_trace_weapon pose = w0;
    fbs_trace_target target;
    fbs_trace_hit hits[4];
    fbs_trace_interval iv;
    size_t n = 12345u;
    int dummy = 0;

    sentinel = (fbs_trace_context *)(void *)&dummy;
    ctx = sentinel;
    CHECK(t, fbs_trace_create(NULL, NULL, NULL) == FBS_TRACE_E_INVALID);
    cfg.max_weapons = 0u;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    cfg = fbs_trace_config_default();
    cfg.max_windows = (1u << 20) + 1u;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    cfg = fbs_trace_config_default();
    cfg.max_struck = 0u;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    cfg = fbs_trace_config_default();
    cfg.dedup = 9;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    cfg = fbs_trace_config_default();
    cfg.policy.max_steps = 1u;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    cfg = fbs_trace_config_default();
    cfg.max_weapons = 1u << 21;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_E_INVALID);
    CHECK(t, ctx == sentinel); /* *out never written on a rejected config */

    cfg = fbs_trace_config_default();
    cfg.max_windows = 2u;
    cfg.policy = fbs_trace_policy_compat_4cm();
    ctx = NULL;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
    if (ctx == NULL) return;

    /* windows */
    win[0].begin = 0.5f;
    win[0].end = 0.25f;
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, win, 1u) == FBS_TRACE_E_INVALID);
    win[0].begin = nan_f;
    win[0].end = 1.0f;
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, win, 1u) == FBS_TRACE_E_INVALID);
    win[0].begin = 0.0f;
    win[0].end = inf_f;
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, win, 1u) == FBS_TRACE_E_INVALID);
    win[0].begin = 0.0f;
    win[0].end = 1.0f;
    win[1] = win[0];
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, win, 3u) == FBS_TRACE_E_INVALID); /* > max_windows */
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 1u) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_interval_get(ctx, 1u, &iv) == FBS_TRACE_E_NOT_FOUND); /* nothing claimed */
    CHECK(t, fbs_trace_begin(NULL, 1u, 1u, NULL, 0) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_begin(ctx, 1u, 1u, win, 2u) == FBS_TRACE_OK);

    /* update */
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, nan_f, &pose) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, inf_f, &pose) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, NULL) == FBS_TRACE_E_INVALID);
    pose.radius = -1.0f;
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &pose) == FBS_TRACE_E_INVALID);
    pose.radius = 0.0f;
    pose.b.y = nan_f;
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &pose) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &w0) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.5f, &w1) == FBS_TRACE_OK);
    CHECK(t, fbs_trace_interval_get(ctx, 1u, NULL) == FBS_TRACE_E_INVALID);
    CHECK(t, fbs_trace_interval_get(NULL, 1u, &iv) == FBS_TRACE_E_INVALID);

    /* targets: *count carries the offending index, nothing else is written */
    target = tgt(1u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                 cap(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.1f));
    {
      fbs_trace_target list[3];
      list[0] = target;
      list[1] = target;
      list[2] = target;
      list[1].zone_id = 1u;
      list[2].zone_id = FBS_TRACE_ZONE_ANY;
      memset(hits, 0xA5, sizeof hits);
      n = 12345u;
      CHECK(t, fbs_trace_test(ctx, 1u, list, 3u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, n == 2u);
      CHECK(t, untouched(hits, sizeof hits, 0xA5));
      list[2].zone_id = 2u;
      list[2].kind = 5;
      n = 0u;
      CHECK(t, fbs_trace_test(ctx, 1u, list, 3u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, n == 2u);
      list[2].kind = (int)FBS_TRACE_TARGET_CAPSULE;
      list[2].current.radius = -1.0f;
      CHECK(t, fbs_trace_test(ctx, 1u, list, 3u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      list[2].current.radius = 0.1f;
      list[2].has_previous = 1;
      list[2].previous.a.x = nan_f;
      CHECK(t, fbs_trace_test(ctx, 1u, list, 3u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, n == 2u);
      CHECK(t, untouched(hits, sizeof hits, 0xA5));
      CHECK(t, fbs_trace_test(ctx, 1u, NULL, 1u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_test(ctx, 1u, list, 1u, NULL, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_test(ctx, 1u, list, 1u, hits, 4u, NULL) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_test(NULL, 1u, list, 1u, hits, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_substeps(ctx, 1u, NULL, 4u, &n) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_substeps(ctx, 1u, NULL, 0u, NULL) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_mark(NULL, 1u, 1u, 1u) == FBS_TRACE_E_INVALID);
      CHECK(t, fbs_trace_is_struck(NULL, 1u, 1u, 1u) == FBS_TRACE_E_INVALID);
    }
    fbs_trace_destroy(ctx);
    fbs_trace_destroy(NULL); /* must be a no-op */
    fbs_trace_reset(NULL);
    CHECK(t, fbs_trace_memory(NULL) == 0u);
  }

  /* status names */
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_OK), "ok") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_CONTACT), "contact") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_INVALID), "invalid") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_CAPACITY), "capacity") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_MEMORY), "memory") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_STATE), "state") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_NOT_FOUND), "not_found") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_FULL), "full") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(FBS_TRACE_E_DEDUP), "dedup") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(42), "unknown") == 0);
  CHECK(t, strcmp(fbs_trace_status_name(-99), "unknown") == 0);
}

/* ------------------------------------------------------------------------- */
/* 13. Capacity and allocator failures                                       */
/* ------------------------------------------------------------------------- */

typedef struct counting_alloc {
  int allocs;
  int frees;
  size_t bytes;
  int fail;
} counting_alloc;

static void *count_alloc(void *user, size_t bytes) {
  counting_alloc *ca = (counting_alloc *)user;
  ca->allocs++;
  ca->bytes = bytes;
  return ca->fail ? NULL : malloc(bytes);
}

static void count_free(void *user, void *ptr) {
  counting_alloc *ca = (counting_alloc *)user;
  ca->frees++;
  free(ptr);
}

/* SAMPLED: a 1000 m translation at spacing 0.04 needs 25000 steps, i.e. 25001
 * samples, far past max_steps = 1024 -> E_CAPACITY with the output untouched.
 * ADVANCE: a point weapon passing the origin sphere (r 0.1) at a standoff of
 * 0.15 never touches; advancement converges towards the closest approach in
 * shrinking steps, so max_steps = 2 is exhausted -> E_CAPACITY. With 64 steps
 * the same case terminates cleanly as "no contact" (t leaves [0,1]). */
static void test_capacity_and_allocator(tally *t) {
  fbs_trace_weapon far0 = wep(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.0f);
  fbs_trace_weapon far1 = wep(v3(1000.0f, 0.0f, 0.0f), v3(1000.0f, 0.0f, 0.0f), 0.0f);
  fbs_trace_weapon graze0 = wep(v3(-1.0f, 0.15f, 0.0f), v3(-1.0f, 0.15f, 0.0f), 0.0f);
  fbs_trace_weapon graze1 = wep(v3(1.0f, 0.15f, 0.0f), v3(1.0f, 0.15f, 0.0f), 0.0f);
  fbs_trace_sphere origin = sph(v3(0.0f, 0.0f, 0.0f), 0.1f);
  fbs_trace_policy sampled = fbs_trace_policy_compat_4cm();
  fbs_trace_policy tight = pol((int)FBS_TRACE_MODE_ADVANCE, 0.04f, 1e-4f, 2u);
  fbs_trace_policy loose = fbs_trace_policy_default();
  fbs_trace_contact c;

  memset(&c, 0xA5, sizeof c);
  CHECK(t, fbs_trace_sweep_sphere(&far0, &far1, &origin, NULL, &sampled, &c) ==
               FBS_TRACE_E_CAPACITY);
  CHECK(t, untouched(&c, sizeof c, 0xA5));
  memset(&c, 0xA5, sizeof c);
  CHECK(t, fbs_trace_sweep_sphere(&graze0, &graze1, &origin, NULL, &tight, &c) ==
               FBS_TRACE_E_CAPACITY);
  CHECK(t, untouched(&c, sizeof c, 0xA5));
  CHECK(t, fbs_trace_sweep_sphere(&graze0, &graze1, &origin, NULL, &loose, &c) == FBS_TRACE_OK);
  CHECK(t, c.contact == 0);
  CHECK(t, c.steps_used > 1u && c.steps_used <= loose.max_steps);
  /* A larger cap makes the same 1000 m sweep succeed. */
  {
    fbs_trace_policy roomy = pol((int)FBS_TRACE_MODE_SAMPLED, 0.04f, 1e-4f, 65536u);
    CHECK(t, fbs_trace_sweep_sphere(&far0, &far1, &origin, NULL, &roomy, &c) ==
                 FBS_TRACE_CONTACT);
    NEAR(t, c.time, 0.0, 1e-9); /* it starts inside the sphere */
  }

  /* allocator failure: *out untouched, no free */
  {
    counting_alloc ca;
    fbs_trace_allocator alloc;
    fbs_trace_context *ctx;
    int dummy = 0;
    fbs_trace_context *sentinel = (fbs_trace_context *)(void *)&dummy;

    memset(&ca, 0, sizeof ca);
    ca.fail = 1;
    alloc.alloc = count_alloc;
    alloc.free = count_free;
    alloc.user = &ca;
    ctx = sentinel;
    CHECK(t, fbs_trace_create(NULL, &alloc, &ctx) == FBS_TRACE_E_MEMORY);
    CHECK(t, ctx == sentinel);
    CHECK(t, ca.allocs == 1 && ca.frees == 0);

    /* exactly one allocation and one free for a working context */
    memset(&ca, 0, sizeof ca);
    ctx = NULL;
    CHECK(t, fbs_trace_create(NULL, &alloc, &ctx) == FBS_TRACE_OK);
    CHECK(t, ca.allocs == 1 && ca.frees == 0);
    if (ctx != NULL) {
      fbs_trace_weapon p0 = wep(v3(-1.0f, -1.0f, 0.0f), v3(-1.0f, 1.0f, 0.0f), 0.0f);
      fbs_trace_weapon p1 = wep(v3(1.0f, -1.0f, 0.0f), v3(1.0f, 1.0f, 0.0f), 0.0f);
      fbs_trace_target target = tgt(1u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                                    cap(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 0.0f, 0.0f), 0.1f));
      fbs_trace_hit hits[4];
      size_t n = 0;
      CHECK(t, fbs_trace_memory(ctx) == ca.bytes);
      CHECK(t, fbs_trace_memory(ctx) > sizeof(fbs_trace_window) * 8u);
      /* the whole session lifecycle allocates nothing further */
      CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &p0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 1.0f, &p1) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_test(ctx, 1u, &target, 1u, hits, 4u, &n) == FBS_TRACE_OK);
      CHECK(t, n == 1u);
      CHECK(t, ca.allocs == 1);
      fbs_trace_destroy(ctx);
    }
    CHECK(t, ca.allocs == 1 && ca.frees == 1);

    /* a NULL alloc/free pair in a non-NULL allocator is rejected */
    alloc.alloc = NULL;
    ctx = sentinel;
    CHECK(t, fbs_trace_create(NULL, &alloc, &ctx) == FBS_TRACE_E_INVALID);
    CHECK(t, ctx == sentinel);
    alloc.alloc = count_alloc;
    alloc.free = NULL;
    CHECK(t, fbs_trace_create(NULL, &alloc, &ctx) == FBS_TRACE_E_INVALID);
  }

  /* no free weapon slot */
  {
    fbs_trace_config cfg = fbs_trace_config_default();
    fbs_trace_context *ctx = NULL;
    cfg.max_weapons = 1u;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
    if (ctx != NULL) {
      CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_begin(ctx, 2u, 1u, NULL, 0) == FBS_TRACE_E_CAPACITY);
      CHECK(t, fbs_trace_begin(ctx, 1u, 2u, NULL, 0) == FBS_TRACE_OK); /* restart is fine */
      CHECK(t, fbs_trace_end(ctx, 1u) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_begin(ctx, 2u, 1u, NULL, 0) == FBS_TRACE_OK); /* slot freed */
      fbs_trace_destroy(ctx);
    }
  }

  /* dedup table exhaustion, from mark and from test */
  {
    fbs_trace_config cfg = fbs_trace_config_default();
    fbs_trace_context *ctx = NULL;
    fbs_trace_weapon p0 = wep(v3(-1.0f, -1.0f, 0.0f), v3(-1.0f, 1.0f, 0.0f), 0.0f);
    fbs_trace_weapon p1 = wep(v3(1.0f, -1.0f, 0.0f), v3(1.0f, 1.0f, 0.0f), 0.0f);
    fbs_trace_target list[2];
    fbs_trace_hit hits[4];
    size_t n = 0;

    cfg.max_struck = 1u;
    cfg.policy = fbs_trace_policy_compat_4cm();
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
    if (ctx != NULL) {
      CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_mark(ctx, 1u, 1u, 0u) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_mark(ctx, 1u, 1u, 0u) == FBS_TRACE_OK); /* idempotent, no growth */
      CHECK(t, fbs_trace_mark(ctx, 1u, 2u, 0u) == FBS_TRACE_E_DEDUP);
      CHECK(t, fbs_trace_is_struck(ctx, 1u, 2u, 0u) == 0);

      CHECK(t, fbs_trace_begin(ctx, 1u, 2u, NULL, 0) == FBS_TRACE_OK); /* clears the table */
      CHECK(t, fbs_trace_update(ctx, 1u, 2u, 0.0f, &p0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 2u, 1.0f, &p1) == FBS_TRACE_OK);
      list[0] = tgt(1u, 0u, (int)FBS_TRACE_TARGET_SPHERE,
                    cap(v3(-0.3f, 0.0f, 0.0f), v3(-0.3f, 0.0f, 0.0f), 0.1f));
      list[1] = tgt(1u, 1u, (int)FBS_TRACE_TARGET_SPHERE,
                    cap(v3(0.3f, 0.0f, 0.0f), v3(0.3f, 0.0f, 0.0f), 0.1f));
      CHECK(t, fbs_trace_test(ctx, 1u, list, 2u, hits, 4u, &n) == FBS_TRACE_E_DEDUP);
      CHECK(t, n == 2u); /* both hits stay written */
      CHECK(t, fbs_trace_is_struck(ctx, 1u, 1u, 0u) == 1);
      CHECK(t, fbs_trace_is_struck(ctx, 1u, 1u, 1u) == 0); /* the unmarked one repeats */
      fbs_trace_destroy(ctx);
    }
  }

  /* substeps capacity: 100 m of weapon travel with max_steps 1024 */
  {
    fbs_trace_config cfg = fbs_trace_config_default();
    fbs_trace_context *ctx = NULL;
    fbs_trace_weapon p0 = wep(v3(0.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f), 0.0f);
    fbs_trace_weapon p1 = wep(v3(100.0f, 0.0f, 0.0f), v3(100.0f, 1.0f, 0.0f), 0.0f);
    fbs_trace_weapon poses[8];
    size_t n = 7u;

    cfg.policy = fbs_trace_policy_compat_4cm();
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
    if (ctx != NULL) {
      CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &p0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 1.0f, &p1) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 8u, &n) == FBS_TRACE_E_CAPACITY);
      CHECK(t, n == 0u);
      /* ADVANCE substeps are always the two end poses */
      fbs_trace_destroy(ctx);
    }
    cfg.policy = fbs_trace_policy_default();
    ctx = NULL;
    CHECK(t, fbs_trace_create(&cfg, NULL, &ctx) == FBS_TRACE_OK);
    if (ctx != NULL) {
      CHECK(t, fbs_trace_begin(ctx, 1u, 1u, NULL, 0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 0.0f, &p0) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_update(ctx, 1u, 1u, 1.0f, &p1) == FBS_TRACE_OK);
      CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 8u, &n) == FBS_TRACE_OK);
      CHECK(t, n == 2u);
      NEAR(t, poses[0].a.x, 0.0, 0.0);
      NEAR(t, poses[1].a.x, 100.0, 0.0);
      /* E_FULL still writes as many entries as fit: cap 1 -> the previous pose */
      memset(poses, 0, sizeof poses);
      CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 1u, &n) == FBS_TRACE_E_FULL);
      CHECK(t, n == 2u);
      NEAR(t, poses[0].a.x, 0.0, 0.0);
      NEAR(t, poses[0].b.y, 1.0, 0.0); /* really the previous pose, not scratch */
      NEAR(t, poses[1].b.y, 0.0, 0.0); /* nothing written past the cap */
      CHECK(t, fbs_trace_substeps(ctx, 1u, poses, 0u, &n) == FBS_TRACE_E_FULL);
      CHECK(t, n == 2u);
      fbs_trace_destroy(ctx);
    }
  }

  /* presets and version */
  {
    fbs_trace_policy compat = fbs_trace_policy_compat_4cm();
    fbs_trace_policy dflt = fbs_trace_policy_default();
    fbs_trace_config cfg = fbs_trace_config_default();
    CHECK(t, compat.mode == (int)FBS_TRACE_MODE_SAMPLED);
    NEAR(t, compat.spacing, 0.04, 1e-7);
    NEAR(t, compat.tolerance, 1e-4, 1e-10);
    CHECK(t, compat.max_steps == 1024u);
    CHECK(t, dflt.mode == (int)FBS_TRACE_MODE_ADVANCE);
    NEAR(t, dflt.spacing, 0.04, 1e-7);
    NEAR(t, dflt.tolerance, 1e-4, 1e-10);
    CHECK(t, dflt.max_steps == 64u);
    CHECK(t, cfg.max_weapons == 4u && cfg.max_windows == 8u && cfg.max_struck == 64u);
    CHECK(t, cfg.dedup == (int)FBS_TRACE_DEDUP_TARGET_ZONE);
    CHECK(t, cfg.policy.mode == dflt.mode && cfg.policy.max_steps == dflt.max_steps);
    CHECK(t, fbs_trace_version() == 100u);
    CHECK(t, fbs_trace_version() == (unsigned)FBS_TRACE_VERSION);
  }
}

/* ------------------------------------------------------------------------- */
/* 14. Fixtures: deterministic cases replayed by the native and WASM tests   */
/* ------------------------------------------------------------------------- */

/* File format (also consumed by integrations/wasm/tests/trace.test.mjs):
 *   {"version":100,"cases":[{"name","kind":"sphere"|"capsule",
 *     "w0":[7],"w1":[7],"t0":[7],"t1":[7]|null,
 *     "policy":[mode,spacing,tolerance,max_steps],
 *     "status":int,"contact":[21]|null}]}
 * Shapes are always 7 floats (ax ay az bx by bz radius); a sphere sets a == b.
 * The contact vector is contact, time, weapon_param, target_param, distance,
 * point(3), target_point(3), normal(3), motion(3), relative_motion(3),
 * steps_used. All numbers are printed with %.9g, which round-trips a float. */

typedef struct fixture_case {
  const char *name;
  int kind; /* 0 sphere, 1 capsule */
  float w0[7], w1[7], t0[7];
  int has_t1;
  float t1[7];
  float policy[4];
} fixture_case;

#define P_SAMPLED {0.0f, 0.04f, 1e-4f, 1024.0f}
#define P_ADVANCE {1.0f, 0.04f, 1e-4f, 64.0f}

static const fixture_case *fixture_cases(size_t *count) {
  static const fixture_case cases[] = {
    /* The 19 mm sphere falls between two 4 cm samples: SAMPLED misses it and
     * ADVANCE finds it (test_thin_target_fast_sweep uses the same geometry). */
    {"sphere_sampled_thin_miss", 0,
     {-1.01f, -.5f, 0.f, -1.01f, .5f, 0.f, 0.f}, {1.01f, -.5f, 0.f, 1.01f, .5f, 0.f, 0.f},
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .019f}, 0, {0}, P_SAMPLED},
    {"sphere_advance_thin_hit", 0,
     {-1.01f, -.5f, 0.f, -1.01f, .5f, 0.f, 0.f}, {1.01f, -.5f, 0.f, 1.01f, .5f, 0.f, 0.f},
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .019f}, 0, {0}, P_ADVANCE},
    {"sphere_sampled_tip", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {1.f, .5f, 0.f, 1.f, .5f, 0.f, .12f}, 0, {0}, P_SAMPLED},
    {"sphere_advance_tip", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {1.f, .5f, 0.f, 1.f, .5f, 0.f, .12f}, 0, {0}, P_ADVANCE},
    {"sphere_sampled_middle", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {.5f, .5f, 0.f, .5f, .5f, 0.f, .12f}, 0, {0}, P_SAMPLED},
    {"sphere_advance_middle", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {.5f, .5f, 0.f, .5f, .5f, 0.f, .12f}, 0, {0}, P_ADVANCE},
    {"capsule_sampled_tip", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {1.f, .5f, -.3f, 1.f, .5f, .3f, .12f}, 0, {0}, P_SAMPLED},
    {"capsule_advance_tip", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {1.f, .5f, -.3f, 1.f, .5f, .3f, .12f}, 0, {0}, P_ADVANCE},
    {"sphere_sampled_rotating", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, .5f, .8660254037844386f, 0.f, 0.f},
     {.75f, .4330127018922193f, 0.f, .75f, .4330127018922193f, 0.f, .05f}, 0, {0}, P_SAMPLED},
    {"sphere_advance_rotating", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, .5f, .8660254037844386f, 0.f, 0.f},
     {.75f, .4330127018922193f, 0.f, .75f, .4330127018922193f, 0.f, .05f}, 0, {0}, P_ADVANCE},
    {"fist_capsule_sampled_overlap", 1,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
     {.1f, -.5f, 0.f, .1f, .5f, 0.f, .2f}, 0, {0}, P_SAMPLED},
    {"fist_capsule_advance_overlap", 1,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
     {.1f, -.5f, 0.f, .1f, .5f, 0.f, .2f}, 0, {0}, P_ADVANCE},
    {"fist_capsule_sampled_clear", 1,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
     {1.f, -.5f, 0.f, 1.f, .5f, 0.f, .2f}, 0, {0}, P_SAMPLED},
    {"fist_capsule_advance_clear", 1,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
     {1.f, -.5f, 0.f, 1.f, .5f, 0.f, .2f}, 0, {0}, P_ADVANCE},
    {"sphere_moving_sampled", 0,
     {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f}, {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f},
     {-1.f, 0.f, 0.f, -1.f, 0.f, 0.f, .1f}, 1, {1.f, 0.f, 0.f, 1.f, 0.f, 0.f, .1f}, P_SAMPLED},
    {"sphere_moving_advance", 0,
     {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f}, {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f},
     {-1.f, 0.f, 0.f, -1.f, 0.f, 0.f, .1f}, 1, {1.f, 0.f, 0.f, 1.f, 0.f, 0.f, .1f}, P_ADVANCE},
    {"capsule_moving_sampled", 1,
     {-1.f, -.5f, 0.f, -1.f, .5f, 0.f, 0.f}, {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f},
     {1.f, -.3f, 0.f, 1.f, .3f, 0.f, .15f}, 1, {.2f, -.3f, 0.f, .2f, .3f, 0.f, .15f}, P_SAMPLED},
    {"capsule_moving_advance", 1,
     {-1.f, -.5f, 0.f, -1.f, .5f, 0.f, 0.f}, {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f},
     {1.f, -.3f, 0.f, 1.f, .3f, 0.f, .15f}, 1, {.2f, -.3f, 0.f, .2f, .3f, 0.f, .15f}, P_ADVANCE},
    {"parallel_axes_sampled", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {0.f, .15f, 0.f, 1.f, .15f, 0.f, .2f}, 0, {0}, P_SAMPLED},
    {"parallel_axes_advance", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {0.f, .15f, 0.f, 1.f, .15f, 0.f, .2f}, 0, {0}, P_ADVANCE},
    {"collinear_clear_sampled", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {2.f, 0.f, 0.f, 3.f, 0.f, 0.f, .2f}, 0, {0}, P_SAMPLED},
    {"collinear_overlap_sampled", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {.5f, 0.f, 0.f, 1.5f, 0.f, 0.f, .1f}, 0, {0}, P_SAMPLED},
    {"collinear_overlap_advance", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {.5f, 0.f, 0.f, 1.5f, 0.f, 0.f, .1f}, 0, {0}, P_ADVANCE},
    {"thick_weapon_sampled", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, .05f}, {0.f, .6f, 0.f, 1.f, .6f, 0.f, .05f},
     {.5f, .9f, -.2f, .5f, .9f, .2f, .1f}, 0, {0}, P_SAMPLED},
    {"thick_weapon_advance", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, .05f}, {0.f, .6f, 0.f, 1.f, .6f, 0.f, .05f},
     {.5f, .9f, -.2f, .5f, .9f, .2f, .1f}, 0, {0}, P_ADVANCE},
    {"sampled_capacity", 0,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {1000.f, 0.f, 0.f, 1000.f, 0.f, 0.f, 0.f},
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .1f}, 0, {0}, P_SAMPLED},
    {"advance_capacity", 0,
     {-1.f, .15f, 0.f, -1.f, .15f, 0.f, 0.f}, {1.f, .15f, 0.f, 1.f, .15f, 0.f, 0.f},
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .1f}, 0, {0}, {1.0f, 0.04f, 1e-4f, 2.0f}},
    {"advance_grazing_clear", 0,
     {-1.f, .15f, 0.f, -1.f, .15f, 0.f, 0.f}, {1.f, .15f, 0.f, 1.f, .15f, 0.f, 0.f},
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .1f}, 0, {0}, P_ADVANCE},
    {"sphere_sampled_far_miss", 0,
     {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, {1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f},
     {0.f, 5.f, 0.f, 0.f, 5.f, 0.f, .1f}, 0, {0}, P_SAMPLED},
    {"sphere_sampled_fine_spacing", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {.5f, .5f, 0.f, .5f, .5f, 0.f, .12f}, 0, {0}, {0.0f, 0.01f, 1e-4f, 1024.0f}},
    {"capsule_degenerate_as_sphere", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 1.f, 0.f, 1.f, 1.f, 0.f, 0.f},
     {.5f, .5f, 0.f, .5f, .5f, 0.f, .12f}, 0, {0}, P_SAMPLED},
    /* Already overlapping at t = 0: contact at time 0 with distance < R. */
    {"sphere_penetrating_start", 0,
     {0.f, -.5f, 0.f, 0.f, .5f, 0.f, 0.f}, {1.f, -.5f, 0.f, 1.f, .5f, 0.f, 0.f},
     {.1f, .2f, 0.f, .1f, .2f, 0.f, .5f}, 0, {0}, P_ADVANCE},
    /* Closest target point is the capsule endpoint (u = 0) and the weapon tip
     * (k = 1), with the axes perpendicular rather than parallel. */
    {"capsule_endpoint_param", 1,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, .5f, 0.f, 1.f, .5f, 0.f, 0.f},
     {1.3f, .6f, 0.f, 1.3f, .6f, 1.f, .5f}, 0, {0}, P_SAMPLED},
    /* Contact only at the inclusive final sample t = 1; see
     * test_endpoint_and_middle_contact for the derivation. */
    {"sphere_sampled_final_sample", 0,
     {0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f}, {0.f, 0.f, .9f, 1.f, 0.f, .9f, 0.f},
     {.5f, 0.f, 1.f, .5f, 0.f, 1.f, .1005f}, 0, {0}, P_SAMPLED}
  };
  *count = sizeof cases / sizeof cases[0];
  return cases;
}

static fbs_trace_weapon weapon_from(const double *f) {
  return wep(v3((float)f[0], (float)f[1], (float)f[2]), v3((float)f[3], (float)f[4], (float)f[5]),
             (float)f[6]);
}

static fbs_trace_capsule capsule_from(const double *f) {
  return cap(v3((float)f[0], (float)f[1], (float)f[2]), v3((float)f[3], (float)f[4], (float)f[5]),
             (float)f[6]);
}

static fbs_trace_sphere sphere_from(const double *f) {
  return sph(v3((float)f[0], (float)f[1], (float)f[2]), (float)f[6]);
}

static int run_fixture(int kind, const double *w0, const double *w1, const double *t0,
                       const double *t1, const double *policy, fbs_trace_contact *out) {
  fbs_trace_weapon a = weapon_from(w0);
  fbs_trace_weapon b = weapon_from(w1);
  fbs_trace_policy p = pol((int)policy[0], (float)policy[1], (float)policy[2],
                           policy[3] < 0.0 ? 0u : (unsigned)policy[3]);
  if (kind == 0) {
    fbs_trace_sphere s0 = sphere_from(t0);
    fbs_trace_sphere s1 = t1 != NULL ? sphere_from(t1) : s0;
    return (int)fbs_trace_sweep_sphere(&a, &b, &s0, t1 != NULL ? &s1 : NULL, &p, out);
  }
  {
    fbs_trace_capsule c0 = capsule_from(t0);
    fbs_trace_capsule c1 = t1 != NULL ? capsule_from(t1) : c0;
    return (int)fbs_trace_sweep_capsule(&a, &b, &c0, t1 != NULL ? &c1 : NULL, &p, out);
  }
}

static void contact_to_floats(const fbs_trace_contact *c, float *o) {
  o[0] = (float)c->contact;
  o[1] = c->time;
  o[2] = c->weapon_param;
  o[3] = c->target_param;
  o[4] = c->distance;
  o[5] = c->point.x;
  o[6] = c->point.y;
  o[7] = c->point.z;
  o[8] = c->target_point.x;
  o[9] = c->target_point.y;
  o[10] = c->target_point.z;
  o[11] = c->normal.x;
  o[12] = c->normal.y;
  o[13] = c->normal.z;
  o[14] = c->motion.x;
  o[15] = c->motion.y;
  o[16] = c->motion.z;
  o[17] = c->relative_motion.x;
  o[18] = c->relative_motion.y;
  o[19] = c->relative_motion.z;
  o[20] = (float)c->steps_used;
}

static void put_floats(FILE *f, const float *v, int n) {
  int i;
  fputc('[', f);
  for (i = 0; i < n; ++i) {
    if (i > 0) fputc(',', f);
    fprintf(f, "%.9g", (double)v[i]);
  }
  fputc(']', f);
}

static void widen(const float *in, double *out, int n) {
  int i;
  for (i = 0; i < n; ++i) out[i] = (double)in[i];
}

static int write_fixtures(const char *path) {
  size_t count = 0, i;
  const fixture_case *cases = fixture_cases(&count);
  FILE *f = fopen(path, "wb");
  if (f == NULL) return 0;
  fprintf(f, "{\n  \"version\": %u,\n  \"cases\": [\n", fbs_trace_version());
  for (i = 0; i < count; ++i) {
    double w0[7], w1[7], t0[7], t1[7], p[4];
    fbs_trace_contact c;
    float out[21];
    int status;
    widen(cases[i].w0, w0, 7);
    widen(cases[i].w1, w1, 7);
    widen(cases[i].t0, t0, 7);
    widen(cases[i].t1, t1, 7);
    widen(cases[i].policy, p, 4);
    memset(&c, 0, sizeof c);
    status = run_fixture(cases[i].kind, w0, w1, t0, cases[i].has_t1 ? t1 : NULL, p, &c);
    fprintf(f, "    {\"name\": \"%s\", \"kind\": \"%s\", \"w0\": ", cases[i].name,
            cases[i].kind == 0 ? "sphere" : "capsule");
    put_floats(f, cases[i].w0, 7);
    fprintf(f, ", \"w1\": ");
    put_floats(f, cases[i].w1, 7);
    fprintf(f, ", \"t0\": ");
    put_floats(f, cases[i].t0, 7);
    fprintf(f, ", \"t1\": ");
    if (cases[i].has_t1) {
      put_floats(f, cases[i].t1, 7);
    } else {
      fprintf(f, "null");
    }
    fprintf(f, ", \"policy\": ");
    put_floats(f, cases[i].policy, 4);
    fprintf(f, ", \"status\": %d, \"contact\": ", status);
    if (status == FBS_TRACE_CONTACT) {
      contact_to_floats(&c, out);
      put_floats(f, out, 21);
    } else {
      fprintf(f, "null");
    }
    fprintf(f, "}%s\n", i + 1u < count ? "," : "");
  }
  fprintf(f, "  ]\n}\n");
  return fclose(f) == 0;
}

/* --- tiny tolerant JSON reader (numbers, strings, arrays, null, objects) --- */

typedef struct json {
  const char *p;
} json;

static void js_skip(json *j) {
  while (*j->p == ' ' || *j->p == '\t' || *j->p == '\n' || *j->p == '\r' || *j->p == ',' ||
         *j->p == ':')
    j->p++;
}

static int js_string(json *j, char *out, size_t cap_bytes) {
  size_t n = 0;
  js_skip(j);
  if (*j->p != '"') return 0;
  j->p++;
  while (*j->p != '\0' && *j->p != '"') {
    if (n + 1u < cap_bytes) out[n] = *j->p;
    n++;
    j->p++;
  }
  if (*j->p != '"') return 0;
  j->p++;
  out[n + 1u < cap_bytes ? n : cap_bytes - 1u] = '\0';
  return 1;
}

static int js_number(json *j, double *out) {
  char *end = NULL;
  js_skip(j);
  *out = strtod(j->p, &end);
  if (end == j->p) return 0;
  j->p = end;
  return 1;
}

static int js_null(json *j) {
  js_skip(j);
  if (strncmp(j->p, "null", 4) == 0) {
    j->p += 4;
    return 1;
  }
  return 0;
}

static int js_numbers(json *j, double *out, int n) {
  int i;
  js_skip(j);
  if (*j->p != '[') return 0;
  j->p++;
  for (i = 0; i < n; ++i) {
    if (!js_number(j, &out[i])) return 0;
  }
  js_skip(j);
  if (*j->p != ']') return 0;
  j->p++;
  return 1;
}

static int js_skip_value(json *j) {
  js_skip(j);
  if (*j->p == '"') {
    char scratch[4];
    return js_string(j, scratch, sizeof scratch);
  }
  if (*j->p == '[' || *j->p == '{') {
    char close = *j->p == '[' ? ']' : '}';
    j->p++;
    for (;;) {
      js_skip(j);
      if (*j->p == close) {
        j->p++;
        return 1;
      }
      if (*j->p == '\0') return 0;
      if (!js_skip_value(j)) return 0;
    }
  }
  if (strncmp(j->p, "null", 4) == 0) {
    j->p += 4;
    return 1;
  }
  if (strncmp(j->p, "true", 4) == 0) {
    j->p += 4;
    return 1;
  }
  if (strncmp(j->p, "false", 5) == 0) {
    j->p += 5;
    return 1;
  }
  {
    double d;
    return js_number(j, &d);
  }
}

typedef struct parsed_case {
  char name[64];
  int kind;
  double w0[7], w1[7], t0[7], t1[7];
  int has_t1;
  double policy[4];
  int status;
  double contact[21];
  int has_contact;
} parsed_case;

static int js_case(json *j, parsed_case *c) {
  char key[24];
  js_skip(j);
  if (*j->p != '{') return 0;
  j->p++;
  memset(c, 0, sizeof *c);
  for (;;) {
    js_skip(j);
    if (*j->p == '}') {
      j->p++;
      return 1;
    }
    if (*j->p == '\0') return 0;
    if (!js_string(j, key, sizeof key)) return 0;
    if (strcmp(key, "name") == 0) {
      if (!js_string(j, c->name, sizeof c->name)) return 0;
    } else if (strcmp(key, "kind") == 0) {
      char kind[16];
      if (!js_string(j, kind, sizeof kind)) return 0;
      c->kind = strcmp(kind, "capsule") == 0 ? 1 : 0;
    } else if (strcmp(key, "w0") == 0) {
      if (!js_numbers(j, c->w0, 7)) return 0;
    } else if (strcmp(key, "w1") == 0) {
      if (!js_numbers(j, c->w1, 7)) return 0;
    } else if (strcmp(key, "t0") == 0) {
      if (!js_numbers(j, c->t0, 7)) return 0;
    } else if (strcmp(key, "t1") == 0) {
      if (js_null(j)) {
        c->has_t1 = 0;
      } else {
        if (!js_numbers(j, c->t1, 7)) return 0;
        c->has_t1 = 1;
      }
    } else if (strcmp(key, "policy") == 0) {
      if (!js_numbers(j, c->policy, 4)) return 0;
    } else if (strcmp(key, "status") == 0) {
      double d;
      if (!js_number(j, &d)) return 0;
      c->status = (int)d;
    } else if (strcmp(key, "contact") == 0) {
      if (js_null(j)) {
        c->has_contact = 0;
      } else {
        if (!js_numbers(j, c->contact, 21)) return 0;
        c->has_contact = 1;
      }
    } else if (!js_skip_value(j)) {
      return 0;
    }
  }
}

static char *slurp(const char *path) {
  FILE *f = fopen(path, "rb");
  long size;
  char *buf;
  if (f == NULL) return NULL;
  if (fseek(f, 0, SEEK_END) != 0) {
    fclose(f);
    return NULL;
  }
  size = ftell(f);
  if (size < 0) {
    fclose(f);
    return NULL;
  }
  rewind(f);
  buf = (char *)malloc((size_t)size + 1u);
  if (buf == NULL) {
    fclose(f);
    return NULL;
  }
  if (fread(buf, 1u, (size_t)size, f) != (size_t)size) {
    free(buf);
    fclose(f);
    return NULL;
  }
  buf[size] = '\0';
  fclose(f);
  return buf;
}

static char *load_fixtures(char *chosen, size_t chosen_cap) {
  static const char *candidates[] = {"fixtures/trace/sweeps.json", "tests/fixtures/trace/sweeps.json",
                                     "../tests/fixtures/trace/sweeps.json"};
  const char *env = getenv("FBS_TRACE_FIXTURES");
  size_t i;
  char *text;
  if (env != NULL && env[0] != '\0') {
    text = slurp(env);
    if (text != NULL) {
      strncpy(chosen, env, chosen_cap - 1u);
      chosen[chosen_cap - 1u] = '\0';
      return text;
    }
  }
  for (i = 0; i < sizeof candidates / sizeof candidates[0]; ++i) {
    text = slurp(candidates[i]);
    if (text != NULL) {
      strncpy(chosen, candidates[i], chosen_cap - 1u);
      chosen[chosen_cap - 1u] = '\0';
      return text;
    }
  }
  return NULL;
}

/* Replays every committed case through the current implementation: the status
 * must match exactly and all 21 contact numbers within 1e-6. */
static void test_fixtures(tally *t) {
  char chosen[512];
  char *text = load_fixtures(chosen, sizeof chosen);
  json j;
  size_t generated = 0;
  int cases_seen = 0;
  int version_seen = 0;
  char key[24];

  (void)fixture_cases(&generated);
  CHECK(t, text != NULL);
  if (text == NULL) {
    printf("      (no fixture file; set FBS_TRACE_FIXTURES or run from tests/)\n");
    return;
  }
  j.p = text;
  js_skip(&j);
  CHECK(t, *j.p == '{');
  if (*j.p != '{') {
    free(text);
    return;
  }
  j.p++;
  for (;;) {
    js_skip(&j);
    if (*j.p == '}' || *j.p == '\0') break;
    if (!js_string(&j, key, sizeof key)) {
      CHECK(t, 0);
      break;
    }
    if (strcmp(key, "version") == 0) {
      double d = 0.0;
      CHECK(t, js_number(&j, &d));
      NEAR(t, d, (double)fbs_trace_version(), 0.0);
      version_seen = 1;
    } else if (strcmp(key, "cases") == 0) {
      js_skip(&j);
      CHECK(t, *j.p == '[');
      if (*j.p != '[') break;
      j.p++;
      for (;;) {
        parsed_case pc;
        fbs_trace_contact c;
        int status;
        js_skip(&j);
        if (*j.p == ']') {
          j.p++;
          break;
        }
        if (*j.p == '\0') {
          CHECK(t, 0);
          break;
        }
        if (!js_case(&j, &pc)) {
          CHECK(t, 0);
          break;
        }
        cases_seen++;
        memset(&c, 0xA5, sizeof c);
        status = run_fixture(pc.kind, pc.w0, pc.w1, pc.t0, pc.has_t1 ? pc.t1 : NULL, pc.policy, &c);
        if (status != pc.status) {
          t->checks++;
          t->failures++;
          printf("FAIL fixture %s: status %s, expected %s\n", pc.name,
                 fbs_trace_status_name(status), fbs_trace_status_name(pc.status));
          continue;
        }
        t->checks++;
        if (status == FBS_TRACE_CONTACT) {
          float got[21];
          int k;
          CHECK(t, pc.has_contact);
          contact_to_floats(&c, got);
          for (k = 0; k < 21; ++k) {
            double a = (double)got[k];
            double b = pc.contact[k];
            int ok = (a - b <= 1e-6) && (b - a <= 1e-6);
            t->checks++;
            if (!ok) {
              t->failures++;
              printf("FAIL fixture %s[%d]: got %.9g want %.9g\n", pc.name, k, a, b);
            }
          }
        } else {
          CHECK(t, !pc.has_contact);
          if (status < 0) CHECK(t, untouched(&c, sizeof c, 0xA5));
        }
      }
    } else if (!js_skip_value(&j)) {
      CHECK(t, 0);
      break;
    }
  }
  CHECK(t, version_seen == 1);
  CHECK(t, cases_seen >= 24);
  CHECK(t, (size_t)cases_seen == generated); /* the committed file is not stale */
  printf("      fixtures: %d cases from %s\n", cases_seen, chosen);
  free(text);
}

/* ------------------------------------------------------------------------- */
/* 15. Timing (reported, not asserted)                                       */
/* ------------------------------------------------------------------------- */

static double bench(int mode, unsigned iterations, double *checksum) {
  fbs_trace_policy p = mode == (int)FBS_TRACE_MODE_SAMPLED ? fbs_trace_policy_compat_4cm()
                                                           : fbs_trace_policy_default();
  fbs_trace_capsule target = cap(v3(0.5f, 0.5f, -0.2f), v3(0.5f, 0.5f, 0.2f), 0.12f);
  clock_t start, stop;
  unsigned i;
  double sum = 0.0;

  start = clock();
  for (i = 0; i < iterations; ++i) {
    /* nudge the sweep every iteration so nothing can be hoisted out */
    float o = (float)((double)(i % 1000u) * 1e-5);
    fbs_trace_weapon w0 = wep(v3(o, 0.0f, 0.0f), v3(1.0f + o, 0.0f, 0.0f), 0.0f);
    fbs_trace_weapon w1 = wep(v3(o, 1.0f, 0.0f), v3(1.0f + o, 1.0f, 0.0f), 0.0f);
    fbs_trace_contact c;
    if (fbs_trace_sweep_capsule(&w0, &w1, &target, NULL, &p, &c) == FBS_TRACE_CONTACT)
      sum += (double)c.time + (double)c.distance;
  }
  stop = clock();
  *checksum = sum;
  return (double)(stop - start) / (double)CLOCKS_PER_SEC;
}

static void test_timing(tally *t) {
  const unsigned n = 100000u;
  double sum_sampled = 0.0, sum_advance = 0.0;
  double secs_sampled = bench((int)FBS_TRACE_MODE_SAMPLED, n, &sum_sampled);
  double secs_advance = bench((int)FBS_TRACE_MODE_ADVANCE, n, &sum_advance);
  printf("      timing: %u SAMPLED capsule sweeps in %.4f s (%.3f us each)\n", n, secs_sampled,
         secs_sampled * 1e6 / (double)n);
  printf("      timing: %u ADVANCE capsule sweeps in %.4f s (%.3f us each)\n", n, secs_advance,
         secs_advance * 1e6 / (double)n);
  CHECK(t, sum_sampled > 0.0);
  CHECK(t, sum_advance > 0.0);
  CHECK(t, secs_sampled >= 0.0 && secs_advance >= 0.0);
}

/* ------------------------------------------------------------------------- */
/* Driver                                                                    */
/* ------------------------------------------------------------------------- */

static void test_bound_capsule(tally *t) {
  fbs_trace_capsule c={{0,0,0},{0,0,0},0};
  CHECK(t,fbs_trace_bind_capsule((fbs_vec3){1,2,3},(fbs_vec3){4,5,6},(fbs_vec3){0,.5f,0},(fbs_vec3){0,-.5f,0},.75f,&c)==FBS_TRACE_OK);
  CHECK(t,c.a.y==2.5f&&c.b.y==4.5f&&c.radius==.75f);
  fbs_trace_capsule old=c;
  CHECK(t,fbs_trace_bind_capsule((fbs_vec3){NAN,0,0},c.b,c.a,c.b,1,&c)==FBS_TRACE_E_INVALID);
  CHECK(t,memcmp(&old,&c,sizeof c)==0);
}

int main(int argc, char **argv) {
  tally t;
  t.checks = 0;
  t.failures = 0;

  if (argc >= 3 && strcmp(argv[1], "--write-fixtures") == 0) {
    size_t count = 0;
    (void)fixture_cases(&count);
    if (!write_fixtures(argv[2])) {
      fprintf(stderr, "could not write %s\n", argv[2]);
      return 1;
    }
    printf("wrote %lu fixture cases to %s\n", (unsigned long)count, argv[2]);
    return 0;
  }
  if (argc >= 2 && strcmp(argv[1], "--write-fixtures") == 0) {
    fprintf(stderr, "usage: %s --write-fixtures <path>\n", argv[0]);
    return 1;
  }

  test_bound_capsule(&t);
  test_thin_target_fast_sweep(&t);
  test_endpoint_and_middle_contact(&t);
  test_rotating_weapon(&t);
  test_zero_length_weapon_held_pose(&t);
  test_target_translation(&t);
  test_parallel_and_collinear(&t);
  test_frame_rate_schedules(&t);
  test_lifecycle(&t);
  test_dual_weapons(&t);
  test_multiple_targets(&t);
  test_duplicates(&t);
  test_malformed_input(&t);
  test_capacity_and_allocator(&t);
  test_fixtures(&t);
  test_timing(&t);

  printf("%d checks passed\n", t.checks - t.failures);
  if (t.failures > 0) printf("%d checks FAILED (of %d)\n", t.failures, t.checks);
  return t.failures;
}
