#ifndef TRACKER_LOCK_ON_H
#define TRACKER_LOCK_ON_H

/**
 * @brief Lock-on target selection for de_tracker (pure logic, no video).
 *
 * The perception stack publishes confirmed observations that carry a stable
 * "trk" (de_common helpers/obs.hpp). de_tracker does no filtering of its
 * own: it is told WHICH track to hold and seeds its pixel tracker from that
 * observation's box.
 *
 *   lockTrk(7)       hold track 7 only; if it vanishes and the SAME id
 *                    reappears the pixel tracker is re-seeded on it
 *   lockClass("x")   hold the first confirmed track of class x (lowest id);
 *                    "" means any class. Once chosen it behaves like
 *                    lockTrk; if that track stays absent for
 *                    class_reacquire_frames observation frames the next
 *                    confirmed track of the class is taken
 *   unlock()         hold nothing
 *
 * onObservations() returns true when the pixel tracker has to be (re)started
 * and fills `seed` with the box to start it on (normalized, top-left, 0..1).
 */

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "../de_common/helpers/obs.hpp"
#include "../de_common/helpers/json_nlohmann.hpp"

namespace de
{
namespace tracker
{

struct LockBox
{
    double x = 0, y = 0, w = 0, h = 0;
};

struct LockConfig
{
    int    class_reacquire_frames = 15;   // absent frames before a class lock takes another track
    int    reseed_cooldown_frames = 10;   // min observation frames between re-seeds while the pixel tracker is not (yet) tracking
    bool   drift_correction = false;      // re-seed when the pixel tracker centre drifts away from the locked track (ai_priority)
    double drift_distance = 0.2;          // fraction of the image diagonal
};

struct PixelTrackerState
{
    bool   tracking = false;              // pixel tracker currently holds a target
    bool   has_center = false;
    double cx = 0.5, cy = 0.5;            // its centre, normalized 0..1
};

/**
 * Box -> wire array [x,y,w,h]: normalized, TOP-LEFT origin, every value in
 * 0..1 with x+w<=1 and y+h<=1 (the same convention as the obs message).
 */
inline nlohmann::json boxToWire (double x, double y, double w, double h)
{
    auto c01 = [](double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); };

    x = c01(x); y = c01(y);
    w = c01(w); h = c01(h);
    if (x + w > 1.0) w = 1.0 - x;
    if (y + h > 1.0) h = 1.0 - y;

    return nlohmann::json::array({x, y, w, h});
}


class CLockOn
{
public:
    enum class Mode { NONE, TRK, CLASS };

    void setConfig (const LockConfig& config) { m_config = config; }

    void lockTrk (const int64_t trk)
    {
        m_mode = Mode::TRK;
        m_trk = trk;
        m_class.clear();
        resetTarget();
    }

    void lockClass (const std::string& cls)
    {
        m_mode = Mode::CLASS;
        m_trk = -1;
        m_class = cls;
        resetTarget();
    }

    void unlock ()
    {
        m_mode = Mode::NONE;
        m_trk = -1;
        m_class.clear();
        resetTarget();
    }

    bool locked () const { return m_mode != Mode::NONE; }
    Mode mode () const { return m_mode; }
    int64_t trk () const { return m_trk; }
    const std::string& cls () const { return m_class; }

    // "none" | "trk:7" | "class:person" | "class:*" (any)
    std::string describe () const
    {
        switch (m_mode)
        {
        case Mode::TRK:   return "trk:" + std::to_string(m_trk);
        case Mode::CLASS: return m_trk >= 0 ? "trk:" + std::to_string(m_trk) : "class:" + (m_class.empty() ? std::string("*") : m_class);
        default:          return "none";
        }
    }

    bool onObservations (const de::ai::ObservationFrame& frame, const PixelTrackerState& tracker, LockBox& seed)
    {
        if (m_mode == Mode::NONE) return false;

        if (m_seeded) ++m_frames_since_seed;

        const de::ai::Observation* target = nullptr;

        if (m_mode == Mode::CLASS && (m_trk < 0 || m_absent_frames >= m_config.class_reacquire_frames))
        {
            const de::ai::Observation* pick = nullptr;
            for (const de::ai::Observation& o : frame.obs)
            {
                if (!o.has_trk) continue;
                if (!m_class.empty() && o.cls != m_class) continue;
                if (o.trk == m_trk) continue;
                if (pick == nullptr || o.trk < pick->trk) pick = &o;
            }

            if (pick != nullptr)
            {
                m_trk = pick->trk;
                resetTarget();
                target = pick;
            }
        }

        if (target == nullptr && m_trk >= 0)
        {
            for (const de::ai::Observation& o : frame.obs)
            {
                if (o.has_trk && o.trk == m_trk) { target = &o; break; }
            }
        }

        if (target == nullptr)
        {
            if (m_trk >= 0) { ++m_absent_frames; m_was_absent = true; }
            return false;
        }

        m_absent_frames = 0;

        bool need = false;
        if (!m_seeded) need = true;
        else if (m_was_absent) need = true;
        else if (!tracker.tracking && m_frames_since_seed >= m_config.reseed_cooldown_frames) need = true;
        else if (tracker.tracking && m_config.drift_correction && tracker.has_center
                 && m_frames_since_seed >= m_config.reseed_cooldown_frames)
        {
            const double dx = (target->x + target->w * 0.5) - tracker.cx;
            const double dy = (target->y + target->h * 0.5) - tracker.cy;
            if (std::sqrt(dx * dx + dy * dy) > std::sqrt(2.0) * m_config.drift_distance) need = true;
        }

        if (!need) return false;

        seed.x = target->x; seed.y = target->y; seed.w = target->w; seed.h = target->h;
        m_seeded = true;
        m_was_absent = false;
        m_frames_since_seed = 0;
        return true;
    }

private:
    void resetTarget ()
    {
        m_seeded = false;
        m_was_absent = false;
        m_absent_frames = 0;
        m_frames_since_seed = 0;
    }

    LockConfig  m_config;
    Mode        m_mode = Mode::NONE;
    int64_t     m_trk = -1;
    std::string m_class;
    bool        m_seeded = false;
    bool        m_was_absent = false;
    int         m_absent_frames = 0;
    int         m_frames_since_seed = 0;
};

}
}

#endif
