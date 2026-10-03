/*
 * fbs/trace.h — FinalBuildSystems event-driven weapon trace (temporal melee
 * contact) library. C99, engine independent, libm only.
 *
 * Two layers:
 *
 *  1. Stateless sweep helpers (fbs_trace_sweep_sphere / fbs_trace_sweep_capsule)
 *     that test a weapon segment moving from a previous pose to a current pose
 *     against a sphere or capsule that may also move. They return the contact
 *     point ON THE WEAPON, the normalized contact time inside the interval and
 *     the material-point motion vector of the struck weapon point.
 *
 *  2. A trace context (fbs_trace_context) that owns per-weapon attack sessions:
 *     begin/update/end/reset lifecycle, previous/current samples, independent
 *     weapon and attack ids, active time windows, duplicate-hit policy, and a
 *     pull-style host collision path (fbs_trace_substeps) for engines that run
 *     their own sweeps. The library never allocates after fbs_trace_create.
 *
 * Units and conventions: the core is unit- and handedness-agnostic; every
 * distance is in the caller's units and every angle-free. The suggested
 * host default is meters, radians, Y-up.
 * Inputs are validated: any non-finite coordinate, negative radius or invalid
 * pointer yields FBS_TRACE_E_INVALID and leaves every output untouched.
 *
 * Thread safety: no globals. Distinct contexts may be used concurrently; one
 * context must not be used from two threads at once.
 */
#ifndef FBS_TRACE_H
#define FBS_TRACE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FBS_TRACE_VERSION_MAJOR 0
#define FBS_TRACE_VERSION_MINOR 1
#define FBS_TRACE_VERSION_PATCH 0
/* Packed as major * 10000 + minor * 100 + patch. */
#define FBS_TRACE_VERSION 100

typedef struct fbs_vec3 {
  float x, y, z;
} fbs_vec3;

/* Status codes. Negative values are errors. On an error no output is written,
 * except where a function documents otherwise (FBS_TRACE_E_FULL and
 * FBS_TRACE_E_DEDUP write partial output; fbs_trace_substeps and
 * fbs_trace_test write *count once the weapon session and inputs are valid). */
typedef enum fbs_trace_status {
  FBS_TRACE_OK = 0,          /* success; for sweeps: no contact in the interval */
  FBS_TRACE_CONTACT = 1,     /* sweeps only: contact found, out filled */
  FBS_TRACE_E_INVALID = -1,  /* NULL pointer, non-finite value, negative radius, bad enum/config value */
  FBS_TRACE_E_CAPACITY = -2, /* motion needs more samples/iterations than policy.max_steps, or no free weapon slot; nothing skipped silently */
  FBS_TRACE_E_MEMORY = -3,   /* allocator returned NULL */
  FBS_TRACE_E_STATE = -4,    /* lifecycle misuse: update without begin, attack id mismatch, time went backwards */
  FBS_TRACE_E_NOT_FOUND = -5,/* weapon id without a session (end, break, interval, substeps, test, mark, is_struck) */
  FBS_TRACE_E_FULL = -6,     /* output array too small; as many entries as fit were written */
  FBS_TRACE_E_DEDUP = -7     /* dedup memory (config.max_struck) exhausted; see fbs_trace_mark / fbs_trace_test */
} fbs_trace_status;

/* Static name of a status code ("ok", "contact", "invalid", ...), never NULL. */
const char *fbs_trace_status_name(int status);

/* ------------------------------------------------------------------------- */
/* Shapes                                                                    */
/* ------------------------------------------------------------------------- */

/* Weapon segment from a (base/hilt) to b (tip) with an optional thickness
 * radius. a == b is a point weapon (fist). radius 0 is a line. */
typedef struct fbs_trace_weapon {
  fbs_vec3 a, b;
  float radius;
} fbs_trace_weapon;

typedef struct fbs_trace_sphere {
  fbs_vec3 center;
  float radius;
} fbs_trace_sphere;

/* Capsule from a (e.g. head) to b (e.g. neck). a == b degenerates to a sphere. */
typedef struct fbs_trace_capsule {
  fbs_vec3 a, b;
  float radius;
} fbs_trace_capsule;

/* Bind a capsule to two evaluated world-space anchors. Offsets are world-axis
 * offsets (not rotated with the anchors). Invalid/overflow output is atomic. */
fbs_trace_status fbs_trace_bind_capsule(fbs_vec3 anchor_a,fbs_vec3 anchor_b,
  fbs_vec3 offset_a,fbs_vec3 offset_b,float radius,fbs_trace_capsule *out);


/* ------------------------------------------------------------------------- */
/* Temporal subdivision policy                                               */
/* ------------------------------------------------------------------------- */

typedef enum fbs_trace_mode {
  /* Uniform samples between the previous and current pose. The sample count
   * is max(1, ceil(travel / spacing)) intervals, samples at i / steps for
   * i = 0..steps inclusive, where travel is the largest endpoint displacement
   * of the weapon plus the largest endpoint displacement of the target. The
   * first sample whose distance is within the combined radius is the contact.
   * spacing = 0.04 matches the 4 cm sampling rule some hosts use; float
   * spacing can add a sample compared with a JavaScript double 0.04. */
  FBS_TRACE_MODE_SAMPLED = 0,
  /* Conservative advancement for linearly moving endpoints and fixed radii:
   * advance by computed separation / (weapon travel + target travel), and
   * report contact once separation <= tolerance (possibly before touching).
   * Uses double arithmetic and a 1e-12 closest-point degeneracy threshold;
   * this is a numerical approximation, not an exact geometric predicate.
   * Exhausting max_steps returns E_CAPACITY with output untouched. */
  FBS_TRACE_MODE_ADVANCE = 1
} fbs_trace_mode;

typedef struct fbs_trace_policy {
  int mode;            /* fbs_trace_mode */
  float spacing;       /* SAMPLED: maximum travel between samples (> 0) */
  float tolerance;     /* ADVANCE: terminate when separation <= tolerance (> 0) */
  unsigned max_steps;  /* resource cap: SAMPLED sample count, ADVANCE iterations (>= 2) */
} fbs_trace_policy;

/* 4 cm compatibility preset for hosts that sample every 4 cm: SAMPLED,
 * spacing 0.04 (4 cm in meters), max_steps 1024. */
fbs_trace_policy fbs_trace_policy_compat_4cm(void);
/* Default error-based preset: ADVANCE, tolerance 1e-4, max_steps 64. */
fbs_trace_policy fbs_trace_policy_default(void);

/* ------------------------------------------------------------------------- */
/* Contact                                                                   */
/* ------------------------------------------------------------------------- */

typedef struct fbs_trace_contact {
  int contact;               /* 1 when a contact was found */
  float time;                /* normalized contact time in [0, 1] within the interval */
  float weapon_param;        /* parameter along the weapon segment (0 = a, 1 = b) at contact */
  float target_param;        /* parameter along the capsule axis at contact (0 for spheres) */
  float distance;            /* closest-point distance between point and target_point at contact time (>= 0); <= R + tolerance under ADVANCE, <= R under SAMPLED; initial overlap may be deeper */
  fbs_vec3 point;            /* contact point on the weapon axis at contact time */
  fbs_vec3 target_point;     /* closest point on the target core (center or axis) at contact time */
  fbs_vec3 normal;           /* unit vector from target_point to point; zero when distance < 1e-12 */
  fbs_vec3 motion;           /* displacement of the struck weapon material point over the whole interval */
  fbs_vec3 relative_motion;  /* motion minus the displacement of the target core point over the interval */
  unsigned steps_used;       /* samples or iterations consumed */
} fbs_trace_contact;

/* Sweep the weapon from w0 to w1 against a sphere moving from s0 to s1. s1 may
 * be NULL for a stationary sphere. The combined radius R = w0->radius +
 * s0->radius is taken from the interval start and not interpolated (the
 * ADVANCE bound has no radius-rate term); w1/s1 radii are validated only.
 * Returns FBS_TRACE_CONTACT and fills out on contact, FBS_TRACE_OK with
 * out->contact = 0 (other fields zero, steps_used set) when the interval is
 * clear, or a negative status with out untouched. */
fbs_trace_status fbs_trace_sweep_sphere(const fbs_trace_weapon *w0,
                                        const fbs_trace_weapon *w1,
                                        const fbs_trace_sphere *s0,
                                        const fbs_trace_sphere *s1,
                                        const fbs_trace_policy *policy,
                                        fbs_trace_contact *out);

/* Same for a capsule moving from c0 to c1 (c1 may be NULL). */
fbs_trace_status fbs_trace_sweep_capsule(const fbs_trace_weapon *w0,
                                         const fbs_trace_weapon *w1,
                                         const fbs_trace_capsule *c0,
                                         const fbs_trace_capsule *c1,
                                         const fbs_trace_policy *policy,
                                         fbs_trace_contact *out);

/* Static closest points between a weapon segment and a capsule (a == b makes a
 * sphere). Fills the contact fields for time 0 and motion zero. Returns
 * FBS_TRACE_CONTACT when the separation is within the combined radius. */
fbs_trace_status fbs_trace_closest(const fbs_trace_weapon *w,
                                   const fbs_trace_capsule *c,
                                   fbs_trace_contact *out);

/* ------------------------------------------------------------------------- */
/* Trace context: sessions, windows, dedup, host-driven substeps             */
/* ------------------------------------------------------------------------- */

typedef struct fbs_trace_allocator {
  void *(*alloc)(void *user, size_t bytes);
  void (*free)(void *user, void *ptr);
  void *user;
} fbs_trace_allocator;

typedef enum fbs_trace_dedup {
  FBS_TRACE_DEDUP_NONE = 0,        /* every interval reports every overlapping target zone */
  FBS_TRACE_DEDUP_TARGET = 1,      /* each target id auto-marked when reported: silent in later calls of the attack */
  FBS_TRACE_DEDUP_TARGET_ZONE = 2, /* each (target, zone) pair auto-marked when reported: silent in later calls of the attack */
  FBS_TRACE_DEDUP_MANUAL = 3       /* nothing auto-marked; host calls fbs_trace_mark */
} fbs_trace_dedup;

typedef struct fbs_trace_config {
  unsigned max_weapons;     /* concurrent weapon sessions (>= 1); default 4 */
  unsigned max_windows;     /* active windows per session (>= 1); default 8 */
  unsigned max_struck;      /* dedup memory entries per session (>= 1); default 64 */
  int dedup;                /* fbs_trace_dedup; default FBS_TRACE_DEDUP_TARGET_ZONE */
  fbs_trace_policy policy;  /* sweep policy used by fbs_trace_test / fbs_trace_substeps */
} fbs_trace_config;

/* Defaults listed above with fbs_trace_policy_default(). */
fbs_trace_config fbs_trace_config_default(void);

typedef struct fbs_trace_context fbs_trace_context;

/* Allocates one block sized by cfg (NULL cfg = defaults; NULL alloc = malloc/free).
 * Returns FBS_TRACE_E_MEMORY on allocation failure with *out untouched. */
fbs_trace_status fbs_trace_create(const fbs_trace_config *cfg,
                                  const fbs_trace_allocator *alloc,
                                  fbs_trace_context **out);
void fbs_trace_destroy(fbs_trace_context *ctx);
/* Drops every session, sample and dedup entry (checkpoint load, teleport, respawn). */
void fbs_trace_reset(fbs_trace_context *ctx);
/* Bytes the context allocated, for host accounting. */
size_t fbs_trace_memory(const fbs_trace_context *ctx);

/* Inclusive attack-time window in the caller's time unit (seconds by convention). */
typedef struct fbs_trace_window {
  float begin, end;
} fbs_trace_window;

/* Start (or restart) the attack session of weapon_id. Clears previous samples
 * and dedup memory for that weapon so no sample from an earlier attack can be
 * connected to the new one. Without windows the whole attack is active. A new
 * weapon_id claims a free slot; FBS_TRACE_E_CAPACITY when all slots are busy. */
fbs_trace_status fbs_trace_begin(fbs_trace_context *ctx, unsigned weapon_id,
                                 unsigned attack_id,
                                 const fbs_trace_window *windows,
                                 size_t window_count);

/* Push the current weapon pose at attack time `time` (non-decreasing within a
 * session; equal times are allowed for held poses). The previous sample of the
 * same session, if any, forms the sweep interval. Returns FBS_TRACE_E_STATE if
 * the weapon has no session or attack_id differs from the session's. */
fbs_trace_status fbs_trace_update(fbs_trace_context *ctx, unsigned weapon_id,
                                  unsigned attack_id, float time,
                                  const fbs_trace_weapon *pose);

/* Discontinuity (teleport, root-motion snap, pose rebind, hit-stop clock
 * rewind): discards the session's previous AND current samples, so the next
 * update is a fresh first sample and may carry any finite time; windows and
 * dedup memory survive. */
fbs_trace_status fbs_trace_break(fbs_trace_context *ctx, unsigned weapon_id);

/* End the session and free its slot. */
fbs_trace_status fbs_trace_end(fbs_trace_context *ctx, unsigned weapon_id);

typedef struct fbs_trace_interval {
  unsigned attack_id;
  int has_previous;         /* 1 when a sweepable interval exists */
  int active;               /* 1 when has_previous and [previous.time, current.time] overlaps a window (or no windows) */
  float previous_time, current_time;
  fbs_trace_weapon previous, current;
} fbs_trace_interval;

/* Inspect the session's current interval. */
fbs_trace_status fbs_trace_interval_get(const fbs_trace_context *ctx,
                                        unsigned weapon_id,
                                        fbs_trace_interval *out);

/* Host-driven collision path: writes the interpolated weapon poses of the
 * active interval according to the context policy (SAMPLED: uniform samples
 * including both ends, spacing derived from the weapon's own travel since the
 * host's targets are unknown here; ADVANCE: both ends only, because exact
 * contact times come from fbs_trace_test). Returns FBS_TRACE_E_FULL if cap is too small,
 * FBS_TRACE_E_CAPACITY if the motion exceeds policy.max_steps (*count = 0).
 * *count is written on every path after argument validation; E_INVALID and
 * E_NOT_FOUND leave it untouched. Inactive intervals produce zero poses and
 * FBS_TRACE_OK. */
fbs_trace_status fbs_trace_substeps(const fbs_trace_context *ctx,
                                    unsigned weapon_id, fbs_trace_weapon *out,
                                    size_t cap, size_t *count);

typedef enum fbs_trace_target_kind {
  FBS_TRACE_TARGET_SPHERE = 0,
  FBS_TRACE_TARGET_CAPSULE = 1
} fbs_trace_target_kind;

#define FBS_TRACE_ZONE_ANY 0xFFFFFFFFu

typedef struct fbs_trace_target {
  unsigned target_id;       /* victim identity used for dedup */
  unsigned zone_id;         /* body zone identity used for per-zone dedup; must not be FBS_TRACE_ZONE_ANY */
  int kind;                 /* fbs_trace_target_kind */
  int has_previous;         /* 1 when `previous` holds the pose at the interval start; 0 = stationary */
  fbs_trace_capsule previous, current; /* spheres use a == b */
} fbs_trace_target;

typedef struct fbs_trace_hit {
  unsigned target_id, zone_id;
  fbs_trace_contact contact;
} fbs_trace_hit;

/* Native narrow phase over the active interval of weapon_id: tests every
 * target, drops those the dedup policy has already marked, auto-marks per
 * policy, and writes all remaining contacts sorted by contact time (ties keep
 * input order). Game policy still chooses which hit becomes damage. Returns
 * FBS_TRACE_OK (count may be 0) or FBS_TRACE_E_FULL when not every hit fits
 * (the earliest `cap` hits are written and *count = cap; hits that did not
 * fit are not marked). FBS_TRACE_E_CAPACITY when any target's motion exceeds
 * the policy cap: no hit is written or marked and *count is the index of the
 * first offending target. FBS_TRACE_E_INVALID for bad target data (nothing
 * written, *count = index of the bad target); targets are validated before
 * the activity check, so bad data is reported even when the interval is
 * inactive. Inactive intervals with valid targets yield 0 hits and OK. */
fbs_trace_status fbs_trace_test(fbs_trace_context *ctx, unsigned weapon_id,
                                const fbs_trace_target *targets,
                                size_t target_count, fbs_trace_hit *hits,
                                size_t cap, size_t *count);

/* Record that (target_id, zone_id) was struck in this attack; zone_id may be
 * FBS_TRACE_ZONE_ANY to silence every zone of the target. Marking an already
 * marked pair is a no-op. FBS_TRACE_E_DEDUP when config.max_struck entries are
 * in use. In fbs_trace_test, auto-marking happens after the hits are written;
 * if it runs out of entries the hits stay written, the unmarked ones will be
 * reported again next interval. FBS_TRACE_E_DEDUP is returned unless the hit
 * output was also truncated, in which case FBS_TRACE_E_FULL takes precedence. */
fbs_trace_status fbs_trace_mark(fbs_trace_context *ctx, unsigned weapon_id,
                                unsigned target_id, unsigned zone_id);
/* 1 when the pair (or the target with FBS_TRACE_ZONE_ANY) is marked, 0 otherwise, negative on error. */
int fbs_trace_is_struck(const fbs_trace_context *ctx, unsigned weapon_id,
                        unsigned target_id, unsigned zone_id);

/* Runtime version (FBS_TRACE_VERSION) for ABI checks across WASM/JS. */
unsigned fbs_trace_version(void);

#ifdef __cplusplus
}
#endif
#endif /* FBS_TRACE_H */
