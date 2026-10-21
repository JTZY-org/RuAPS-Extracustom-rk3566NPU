#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

// 人物跟踪回传固定 11 字节：0xFD 0x01。
// UserAppExChange 先跑 Python，YOLO 回调之后才 push 0xFE。
// 若宿主 pushBroadcastData 只保留最后一包，Python 里先推的 FD 会被后面的 FE 盖掉，
// 地面站就会一直停在「等待回传」。
// 跟踪开关和速度仍在 user_app.py。这里只把 Python 刚推过的那一包记下来，
// 等 ProtocolSerializer::broadcast 把本帧所有 0xFE 推完后再补推一次。
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
            // Byte2 bit0 = 跟踪开。开着时每个 YOLO 回调都在 FE 后面再补一包，
            // 这样下一拍的 FE 盖不住最新的 FD。停止帧 bit0 为 0，只补推一次。
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
