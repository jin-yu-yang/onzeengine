// Core ids, vectors, and constants
#pragma once

#include <cstdint>

namespace onze {
    using Tick = std::uint32_t;     // 60 per second
    using PlayerId = std::uint8_t;  // 0..21; 0-10 players home, 11-21 players away
    using TeamId = std::uint8_t;    // 0 home, 1 away

    inline constexpr std::uint8_t kNone = 0xFF;
    inline constexpr int kPlayers = 22;
    inline constexpr int kTeamSize = 11;
    inline constexpr int kTicksPerSecond = 60;
    inline constexpr int kSubSteps = 4;             // ball sub-step per tick; 240 Hz
    inline constexpr int kDecisionPeriod = 6;       // brain runs every 6 tick; 10 Hz
    inline constexpr double kTickDt = 1.0 / 60.0;
    inline constexpr double kSubDt = 1.0 / 240.0;

    constexpr TeamId team_of(PlayerId p) {return static_cast<TeamId>(p / kTeamSize);}
    constexpr int slot_of(PlayerId p) {return p % kTeamSize;}
    constexpr PlayerId player_id(TeamId t, int slot) {return static_cast<PlayerId>(t * kTeamSize + slot); }

    // state vectors
    struct Vec2 {
        float x, y;
    };
    struct Vec3 {
        float x, y, z;
    };

    // computational vectors
    struct Vec2d {
        double x = 0.0, y = 0.0;
        };
    struct Vec3d {
        double x = 0.0, y = 0.0, z = 0.0;
    };

    constexpr Vec2d operator+(Vec2d a, Vec2d b) { return {a.x + b.x, a.y + b.y}; }
    constexpr Vec2d operator-(Vec2d a, Vec2d b) { return {a.x - b.x, a.y - b.y}; }
    constexpr Vec2d operator*(Vec2d a, double s) { return {a.x * s, a.y * s}; }
    constexpr Vec2d operator*(double s, Vec2d a) { return a * s; }
    constexpr double dot(Vec2d a, Vec2d b) { return a.x * b.x + a.y * b.y; }
    constexpr double cross(Vec2d a, Vec2d b) { return a.x * b.y - a.y * b.x; }

    constexpr Vec3d operator+(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
    constexpr Vec3d operator-(Vec3d a, Vec3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
    constexpr Vec3d operator*(Vec3d a, double s) { return {a.x * s, a.y * s, a.z * s}; }
    constexpr Vec3d operator*(double s, Vec3d a) { return a * s; }
    constexpr double dot(Vec3d a, Vec3d b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    constexpr Vec3d cross(Vec3d a, Vec3d b) { 
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; 
    }

    constexpr Vec2d to_d(Vec2 v) { return {v.x, v.y}; }
    constexpr Vec3d to_d(Vec3 v) { return {v.x, v.y, v.z}; }
    constexpr Vec2 to_f(Vec2d v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }
    constexpr Vec3 to_f(Vec3d v) { 
        return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
    }
    constexpr Vec2d xy(Vec3d v) { return {v.x, v.y}; }
    
} // namespace onze