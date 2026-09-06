#pragma once

#include "PhysecsAPI.h"
#include <entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Constraint1DFlags.h>
#include <SIMD.h>

namespace physecs {

    struct MassData;
    struct Constraint1DWriter;
    struct Constraint1DLayout;
    struct Constraint1DReader;

    struct JointWorldSpaceData {
        glm::vec3 p0;
        glm::vec3 p1;
        glm::vec3 r0;
        glm::vec3 r1;
        glm::mat3 u0;
        glm::mat3 u1;
    };

    struct alignas(16) Constraint1DDescriptor {
        glm::vec3 linear0 = glm::vec3(0.f);
        float stiffness = 0.f;
        glm::vec3 linear1 = glm::vec3(0.f);
        float damping = 0.f;
        glm::vec3 angular0 = glm::vec3(0.f);
        float targetVelocity = 0.f;
        glm::vec3 angular1 = glm::vec3(0.f);
        float geometricError = 0.f;
        float minForce = std::numeric_limits<float>::lowest();
        float maxForce = std::numeric_limits<float>::max();
    };

    struct Constraint1DWriterContext {
        Mat3V* invR0 = nullptr;
        Mat3V* invR1 = nullptr;
        MassData* massData0 = nullptr;
        MassData* massData1 = nullptr;
    };

    typedef void (*MakeConstraintsFunc)(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints);

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

    template<int Flags, int Count = 1, auto Lambdas = nullptr, auto Gate = nullptr>
    struct ConstraintBlock {
        static constexpr int  flags   = Flags;
        static constexpr int  count   = Count;
        static constexpr auto lambdas = Lambdas;
        static constexpr auto gate    = Gate;

        static constexpr bool isHardEquality = !(Flags & SOFT || Flags & LIMITED);
        static constexpr bool isHardEqualityAngular = isHardEquality && (Flags & ANGULAR);
    };

    template<typename... Blocks>
    struct ConstraintLayout {
        static constexpr int count = (0 + ... + Blocks::count);
        static constexpr int hardEqualityCount = (0 + ... + (Blocks::isHardEquality ? Blocks::count : 0));
        static constexpr int hardEqualityAngularCount = (0 + ... + (Blocks::isHardEqualityAngular ? Blocks::count : 0));
    };

    template<typename Impl, typename Layout, typename Cache, typename Data>
    class JointImpl : public Joint {
    protected:

        [[no_unique_address]] Cache cache;
        [[no_unique_address]] Data data;

        void prepare(const entt::registry&) {}

    private:
        static void makeFinalConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints);

        template<typename Block>
        float* getLambdas();
        template<typename Block>
        void createConstraints(Constraint1DLayout& constraintLayout);
        template<typename... Blocks>
        void createConstraints(ConstraintLayout<Blocks...>, Constraint1DLayout& constraintLayout);
        template<typename Block>
        void storeAccumulatedImpulses(Constraint1DReader& constraints);
        template<typename... Blocks>
        void storeAccumulatedImpulses(ConstraintLayout<Blocks...>, Constraint1DReader& constraints);

    public:
        using Joint::Joint;
        JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) override;
        void storeAccumulatedImpulses(Constraint1DReader &constraints) override;
    };

#define PHYSECS_DECLARE_JOINT_IMPL(joint) \
    extern template class JointImpl<joint, joint##Def::Layout, joint##Def::Cache, joint##Def::Data>

#define PHYSECS_DEFINE_JOINT_IMPL(joint) \
    template class JointImpl<joint, joint##Def::Layout, joint##Def::Cache, joint##Def::Data>
}
