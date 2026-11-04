#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

// Person-track feedback is a fixed 11-byte packet: 0xFD 0x01.
// UserAppExChange runs Python first; YOLO callbacks push 0xFE later.
// If the host pushBroadcastData keeps only the last packet, the FD pushed
// from Python is overwritten by a later FE and the ground station stays
// stuck waiting for feedback.
// Track enable/speed stay in user_app.py. Here we only remember the packet
// Python just pushed, then re-push it after ProtocolSerializer::broadcast
// has flushed every 0xFE for this frame.
class TrackFeedbackRelay
{
public:
    static void noteFromPython(const std::vector<uint8_t> &packet)
    {
        if (packet.size() != 11 || packet[0] != 0xFD || packet[1] != 0x01)
        {
            return;
        }
        State &s = state();
        std::lock_guard<std::mutex> lock(s.mu);
        s.latest = packet;
        s.pending = true;
    }

    static void replayAfterDetections(const std::function<void(std::vector<uint8_t>)> &pushCallback)
    {
        if (!pushCallback)
        {
            return;
        }

        std::vector<uint8_t> copy;
        {
            State &s = state();
            std::lock_guard<std::mutex> lock(s.mu);
            if (!s.pending || s.latest.size() != 11)
            {
                return;
            }
            copy = s.latest;
            // Byte2 bit0 = tracking on. While on, every YOLO callback re-pushes
            // after FE so the next FE cannot bury the latest FD. Stop frames
            // have bit0 clear and are re-pushed only once.
            const bool trackingOn = (copy[2] & 0x01) != 0;
            if (!trackingOn)
            {
                s.pending = false;
            }
        }
        pushCallback(std::move(copy));
    }

private:
    struct State
    {
        std::mutex mu;
        std::vector<uint8_t> latest;
        bool pending = false;
    };

    static State &state()
    {
        static State s;
        return s;
    }
};
