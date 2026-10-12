/**
 * @file test_lock_on.cpp
 * @brief standalone tests for src/tracker/lock_on.hpp (lock-on target
 *        selection and the top-left box convention).
 *
 * Build & run:  make -C test && ./test/test_lock_on
 */

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../src/tracker/lock_on.hpp"

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "ASSERTION FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_FALSE(condition) ASSERT_TRUE(!(condition))
#define ASSERT_NEAR(a, b) ASSERT_TRUE(std::fabs((a) - (b)) < 1e-9)

using de::ai::Observation;
using de::ai::ObservationFrame;
using de::tracker::CLockOn;
using de::tracker::LockBox;
using de::tracker::LockConfig;
using de::tracker::PixelTrackerState;

static Observation ob (int64_t trk, const std::string& cls, double x, double y, double w = 0.1, double h = 0.2)
{
    Observation o;
    o.has_trk = true; o.trk = trk;
    o.cls = cls; o.conf = 0.9;
    o.x = x; o.y = y; o.w = w; o.h = h;
    return o;
}

static ObservationFrame fr (const std::vector<Observation>& obs)
{
    ObservationFrame f;
    f.cam_id = "CAM"; f.tm_capture = 1; f.obs = obs;
    return f;
}

static PixelTrackerState tracking () { PixelTrackerState s; s.tracking = true; return s; }
static PixelTrackerState idle () { return PixelTrackerState(); }


bool test_lock_trk_seeds_at_its_box ()
{
    std::cout << "Testing: lock {trk:7} seeds the tracker at trk 7's box..." << std::endl;

    CLockOn lock;
    LockBox seed;

    // nothing locked: never seeds
    ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed));

    lock.lockTrk(7);
    ASSERT_TRUE(lock.locked());
    ASSERT_TRUE(lock.describe() == "trk:7");

    // other tracks are ignored
    ASSERT_FALSE(lock.onObservations(fr({ ob(3, "person", 0.1, 0.1), ob(9, "car", 0.7, 0.7) }), idle(), seed));

    ASSERT_TRUE(lock.onObservations(fr({ ob(3, "person", 0.1, 0.1), ob(7, "person", 0.40, 0.30, 0.12, 0.25) }), idle(), seed));
    ASSERT_NEAR(seed.x, 0.40); ASSERT_NEAR(seed.y, 0.30);
    ASSERT_NEAR(seed.w, 0.12); ASSERT_NEAR(seed.h, 0.25);

    // already seeded and the pixel tracker holds it: no more seeds
    for (int i = 0; i < 20; ++i)
    {
        ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.41 + 0.001 * i, 0.30) }), tracking(), seed));
    }

    std::cout << "passed" << std::endl;
    return true;
}


bool test_vanish_and_reappear_reseeds ()
{
    std::cout << "Testing: trk 7 vanishing then reappearing re-seeds..." << std::endl;

    CLockOn lock;
    LockBox seed;
    lock.lockTrk(7);

    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.40, 0.30) }), idle(), seed));
    ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.41, 0.30) }), tracking(), seed));

    // gone for a while (the pixel tracker may still be "tracking" something)
    for (int i = 0; i < 4; ++i)
    {
        ASSERT_FALSE(lock.onObservations(fr({ ob(3, "person", 0.1, 0.1) }), tracking(), seed));
    }

    // same id back, somewhere else: seed there
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.62, 0.55) }), tracking(), seed));
    ASSERT_NEAR(seed.x, 0.62); ASSERT_NEAR(seed.y, 0.55);

    // a different id never takes over a trk lock
    for (int i = 0; i < 40; ++i)
    {
        ASSERT_FALSE(lock.onObservations(fr({ ob(8, "person", 0.62, 0.55) }), idle(), seed));
    }

    std::cout << "passed" << std::endl;
    return true;
}


bool test_lost_pixel_tracker_reseeds_after_cooldown ()
{
    std::cout << "Testing: lost pixel tracker re-seeds, but not before the cooldown..." << std::endl;

    LockConfig c; c.reseed_cooldown_frames = 5;
    CLockOn lock; lock.setConfig(c);
    LockBox seed;
    lock.lockTrk(7);

    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed));

    // the tracker needs a few frames to come up: no re-seed storm
    int reseeds = 0;
    for (int i = 0; i < 4; ++i) reseeds += lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed) ? 1 : 0;
    ASSERT_TRUE(reseeds == 0);

    // still not tracking after the cooldown: seed again
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed));

    std::cout << "passed" << std::endl;
    return true;
}


bool test_lock_class_picks_first_confirmed_track ()
{
    std::cout << "Testing: lock {class} takes the first confirmed track (lowest id)..." << std::endl;

    CLockOn lock;
    LockBox seed;
    lock.lockClass("person");
    ASSERT_TRUE(lock.describe() == "class:person");

    // a car is not a person
    ASSERT_FALSE(lock.onObservations(fr({ ob(1, "car", 0.5, 0.5) }), idle(), seed));

    ASSERT_TRUE(lock.onObservations(fr({ ob(9, "person", 0.7, 0.2), ob(4, "person", 0.2, 0.6), ob(2, "car", 0.5, 0.5) }), idle(), seed));
    ASSERT_NEAR(seed.x, 0.2); ASSERT_NEAR(seed.y, 0.6);
    ASSERT_TRUE(lock.trk() == 4);
    ASSERT_TRUE(lock.describe() == "trk:4");

    // it sticks to trk 4 even when a lower id shows up later
    ASSERT_FALSE(lock.onObservations(fr({ ob(3, "person", 0.1, 0.1), ob(4, "person", 0.21, 0.6) }), tracking(), seed));
    ASSERT_TRUE(lock.trk() == 4);

    // class "" = any class (what TRACKING_ENABLE arms)
    CLockOn any;
    any.lockClass("");
    ASSERT_TRUE(any.describe() == "class:*");
    ASSERT_TRUE(any.onObservations(fr({ ob(6, "dog", 0.3, 0.3), ob(5, "car", 0.6, 0.6) }), idle(), seed));
    ASSERT_TRUE(any.trk() == 5);

    std::cout << "passed" << std::endl;
    return true;
}


bool test_class_lock_reacquires_after_long_absence ()
{
    std::cout << "Testing: a class lock takes another track after class_reacquire_frames..." << std::endl;

    LockConfig c; c.class_reacquire_frames = 3;
    CLockOn lock; lock.setConfig(c);
    LockBox seed;
    lock.lockClass("person");
    ASSERT_TRUE(lock.onObservations(fr({ ob(4, "person", 0.2, 0.2) }), idle(), seed));

    // trk 4 gone; a new person (id 11) is around; take it only after 3 absent frames
    ASSERT_FALSE(lock.onObservations(fr({ ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_FALSE(lock.onObservations(fr({ ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_FALSE(lock.onObservations(fr({ ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_TRUE(lock.onObservations(fr({ ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_TRUE(lock.trk() == 11);
    ASSERT_NEAR(seed.x, 0.7);

    // but if the old id comes back in time it is kept
    CLockOn k; k.setConfig(c);
    k.lockClass("person");
    ASSERT_TRUE(k.onObservations(fr({ ob(4, "person", 0.2, 0.2) }), idle(), seed));
    ASSERT_FALSE(k.onObservations(fr({ ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_TRUE(k.onObservations(fr({ ob(4, "person", 0.25, 0.2), ob(11, "person", 0.7, 0.7) }), tracking(), seed));
    ASSERT_TRUE(k.trk() == 4);

    std::cout << "passed" << std::endl;
    return true;
}


bool test_unlock_and_relock ()
{
    std::cout << "Testing: unlock stops seeding; relocking starts fresh..." << std::endl;

    CLockOn lock;
    LockBox seed;
    lock.lockTrk(7);
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed));

    lock.unlock();
    ASSERT_FALSE(lock.locked());
    ASSERT_TRUE(lock.describe() == "none");
    ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.3) }), idle(), seed));

    lock.lockTrk(7);
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.45, 0.3) }), idle(), seed));
    ASSERT_NEAR(seed.x, 0.45);

    std::cout << "passed" << std::endl;
    return true;
}


bool test_drift_correction ()
{
    std::cout << "Testing: ai_priority drift correction..." << std::endl;

    LockConfig c; c.drift_correction = true; c.drift_distance = 0.2; c.reseed_cooldown_frames = 2;
    CLockOn lock; lock.setConfig(c);
    LockBox seed;
    lock.lockTrk(7);
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), idle(), seed));

    PixelTrackerState near_t = tracking(); near_t.has_center = true; near_t.cx = 0.46; near_t.cy = 0.46;
    PixelTrackerState far_t  = tracking(); far_t.has_center = true;  far_t.cx = 0.10; far_t.cy = 0.90;

    // within cooldown: nothing
    ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), far_t, seed));
    // after cooldown: close -> keep, far -> re-seed
    ASSERT_FALSE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), near_t, seed));
    ASSERT_TRUE(lock.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), far_t, seed));

    // disabled: never
    CLockOn off;
    off.lockTrk(7);
    ASSERT_TRUE(off.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), idle(), seed));
    for (int i = 0; i < 30; ++i) ASSERT_FALSE(off.onObservations(fr({ ob(7, "person", 0.4, 0.4, 0.1, 0.1) }), far_t, seed));

    std::cout << "passed" << std::endl;
    return true;
}


bool test_output_box_is_top_left_0_1 ()
{
    std::cout << "Testing: tracker output box uses TOP-LEFT [0..1]..." << std::endl;

    nlohmann::json b = de::tracker::boxToWire(0.25, 0.35, 0.10, 0.20);
    ASSERT_TRUE(b.is_array() && b.size() == 4);
    ASSERT_NEAR(b[0].get<double>(), 0.25);   // x of the top-left corner
    ASSERT_NEAR(b[1].get<double>(), 0.35);
    ASSERT_NEAR(b[2].get<double>(), 0.10);   // width, not x2
    ASSERT_NEAR(b[3].get<double>(), 0.20);

    // never outside the frame
    nlohmann::json c = de::tracker::boxToWire(-0.05, 0.95, 0.30, 0.30);
    ASSERT_NEAR(c[0].get<double>(), 0.0);
    ASSERT_NEAR(c[1].get<double>(), 0.95);
    ASSERT_TRUE(c[0].get<double>() + c[2].get<double>() <= 1.0 + 1e-12);
    ASSERT_TRUE(c[1].get<double>() + c[3].get<double>() <= 1.0 + 1e-12);

    // and a valid obs-contract box
    de::ai::Observation o;
    o.cls = "x"; o.conf = 1.0;
    o.x = c[0].get<double>(); o.y = c[1].get<double>(); o.w = c[2].get<double>(); o.h = c[3].get<double>();
    ASSERT_TRUE(de::ai::obsBoxValid(o));

    std::cout << "passed" << std::endl;
    return true;
}


int main ()
{
    bool ok = true;

    ok &= test_lock_trk_seeds_at_its_box();
    ok &= test_vanish_and_reappear_reseeds();
    ok &= test_lost_pixel_tracker_reseeds_after_cooldown();
    ok &= test_lock_class_picks_first_confirmed_track();
    ok &= test_class_lock_reacquires_after_long_absence();
    ok &= test_unlock_and_relock();
    ok &= test_drift_correction();
    ok &= test_output_box_is_top_left_0_1();

    if (ok)
    {
        std::cout << std::endl << "ALL TESTS PASSED" << std::endl;
        return 0;
    }

    std::cout << std::endl << "TESTS FAILED" << std::endl;
    return 1;
}
