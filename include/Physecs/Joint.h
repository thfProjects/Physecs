#pragma once

#include "PhysecsAPI.h"
#include <entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Constraint1DContainer.h>

namespace physecs {

    struct JointWorldSpaceData {
        glm::vec3 p0;
        glm::vec3 p1;
        glm::vec3 r0;
        glm::vec3 r1;
        glm::mat3 u0;
        glm::mat3 u1;
    };

    typedef void (*MakeConstraintsFunc)(const JointWorldSpaceData& worldSpaceData, void* additionalData, Constraint1DWriter& constraints);

    struct JointSolverData {
        int b0;
        int b1;
        glm::vec3 r0;
        glm::vec3 r1;
        glm::mat3 u0;
        glm::mat3 u1;
        void* additionalData;
        MakeConstraintsFunc makeConstraintsFunc;
    };

    struct JointSolverDesc {
        void* additionalData;
        MakeConstraintsFunc makeConstraintsFunc;
    };

    class PHYSECS_API Joint {
        int color = -1;
    protected:
        entt::entity entity0;
        entt::entity entity1;
        glm::vec3 anchor0Pos;
        glm::quat anchor0Or;
        glm::vec3 anchor1Pos;
        glm::quat anchor1Or;

    public:
        Joint(entt::entity entity0, glm::vec3 anchor0Pos, glm::quat anchor0Or, entt::entity entity1, glm::vec3 anchor1Pos, glm::quat anchor1Or) :
        entity0(entity0),
        entity1(entity1),
        anchor0Pos(anchor0Pos),
        anchor0Or(anchor0Or),
        anchor1Pos(anchor1Pos),
        anchor1Or(anchor1Or) {}

        virtual JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout& constraintLayout) = 0;
        virtual void storeAccumulatedImpulses(Constraint1DReader& constraints) = 0;

        entt::entity getEntity0() const { return entity0; }
        entt::entity getEntity1() const { return entity1; }
        glm::vec3 getAnchor0Pos() const { return anchor0Pos; }
        glm::quat getAnchor0Or() const { return anchor0Or; }
        glm::vec3 getAnchor1Pos() const { return anchor1Pos; }
        glm::quat getAnchor1Or() const { return anchor1Or; }
        void setColor(int color) { this->color = color; }
        int getColor() const { return color; }

        virtual ~Joint() = default;
    };

    struct EmptyData {};
    struct EmptyCache {};

    template<int Flags, int Count = 1, auto Lambdas = nullptr, auto Gate = nullptr>
    struct ConstraintBlock {
        static constexpr int  flags   = Flags;
        static constexpr int  count   = Count;
        static constexpr auto lambdas = Lambdas;
        static constexpr auto gate    = Gate;
    };

    template<typename... Blocks>
    struct ConstraintLayout {};

    template<typename Impl, typename Layout, typename Cache = EmptyCache, typename Data = EmptyData>
    class JointImpl : public Joint {
    protected:
        [[no_unique_address]] Cache cache;
        [[no_unique_address]] Data data;

        void prepare(const entt::registry&) {}

    private:
        template<typename... Blocks, typename F>
        constexpr static void forEachBlock(ConstraintLayout<Blocks...>, F&& f) { (f.template operator()<Blocks>(), ...); }

        template<typename Block>
        constexpr float* getLambdas() {
            if constexpr (Block::lambdas == nullptr) return nullptr;
            else if constexpr (std::is_array_v<std::remove_reference_t<decltype(cache.*Block::lambdas)>>) {
                return cache.*Block::lambdas;
            }
            else return &(cache.*Block::lambdas);
        }

    public:
        using Joint::Joint;
        JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) override;
        void storeAccumulatedImpulses(Constraint1DReader &constraints) override;
    };

    template<typename Impl, typename Layout, typename Cache, typename Data>
    JointSolverDesc JointImpl<Impl, Layout, Cache, Data>::getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) {
        static_cast<Impl*>(this)->prepare(registry);
        forEachBlock(Layout{}, [&]<typename Block>{
            if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
            constraintLayout.createConstraints<Block::flags, Block::count>(getLambdas<Block>());
        });
        return JointSolverDesc{ &data, Impl::makeConstraints };
    }

    template<typename Impl, typename Layout, typename Cache, typename Data>
    void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(Constraint1DReader &constraints) {
        forEachBlock(Layout{}, [&]<typename Block>{
            if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
            for (int i = 0; i < Block::count; ++i) {
                const float lambda = constraints.nextTotalLambda<Block::flags>();
                if constexpr (Block::lambdas != nullptr) {
                    getLambdas<Block>()[i] = lambda;
                }
            }
        });
    }
}
