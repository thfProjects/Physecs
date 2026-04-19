#pragma once

#include <glm\glm.hpp>
#include <mutex>
#include <vector>

namespace physecs {

    struct DebugDrawPoint {
        glm::vec3 position;
        glm::vec3 color;
    };

    struct DebugDrawLine {
        glm::vec3 start;
        glm::vec3 end;
        glm::vec3 color;
    };

    namespace Color {
        constexpr glm::vec3 WHITE = glm::vec3(1.f, 1.f, 1.f);
        constexpr glm::vec3 RED = glm::vec3(1.f, 0.f, 0.f);
        constexpr glm::vec3 GREEN = glm::vec3(0.f, 1.f, 0.f);
        constexpr glm::vec3 BLUE = glm::vec3(0.f, 0.f, 1.f);
    }

    class DebugDrawContext {
        std::mutex mutex;
        std::vector<DebugDrawPoint> points;
        std::vector<DebugDrawLine> lines;

    public:
        void clear() {
            points.clear();
            lines.clear();
        }

        void addPoint(const glm::vec3& position, const glm::vec3& color) {
            std::unique_lock lock(mutex);
            points.emplace_back(position, color);
        }

        void addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color) {
            std::unique_lock lock(mutex);
            lines.emplace_back(start, end, color);
        }

        const std::vector<DebugDrawPoint>& getPoints() const { return points; }
        const std::vector<DebugDrawLine>& getLines() const { return lines; }
    };
}