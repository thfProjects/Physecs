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

    struct Constraint1DDescriptor {
        glm::vec3 linear0 = glm::vec3(0.f);
        glm::vec3 linear1 = glm::vec3(0.f);
        glm::vec3 angular0 = glm::vec3(0.f);
        glm::vec3 angular1 = glm::vec3(0.f);
        float geometricError = 0.f;
        float targetVelocity = 0.f;
        float stiffness = 0.f;
        float damping = 0.f;
        float minForce = std::numeric_limits<float>::lowest();
        float maxForce = std::numeric_limits<float>::max();
    };

    struct Constraint1DWriterContext {
        glm::mat3* r0 = nullptr;
        glm::mat3* r1 = nullptr;
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
        static void makeFinalConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints);

        [[no_unique_address]] Cache cache;
        [[no_unique_address]] Data data;

        void prepare(const entt::registry&) {}

    private:
        template<typename Block>
        __forceinline float* getLambdas() {
            if constexpr (Block::lambdas == nullptr) return nullptr;
            else if constexpr (std::is_array_v<std::remove_reference_t<decltype(cache.*Block::lambdas)>>) {
                return cache.*Block::lambdas;
            }
            else return &(cache.*Block::lambdas);
        }

        template<typename Block>
        __forceinline void createConstraints(Constraint1DLayout& constraintLayout) {
            if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
            constraintLayout.createConstraints<Block::flags, Block::count>(getLambdas<Block>());
        }

        template<typename... Blocks>
        __forceinline void createConstraints(ConstraintLayout<Blocks...>, Constraint1DLayout& constraintLayout) {
            (createConstraints<Blocks>(constraintLayout), ...);
        }

        template<typename Block>
        __forceinline void storeAccumulatedImpulses(Constraint1DReader& constraints) {
            if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
            for (int i = 0; i < Block::count; ++i) {
                const float lambda = constraints.nextTotalLambda<Block::flags>();
                if constexpr (Block::lambdas != nullptr) getLambdas<Block>()[i] = lambda;
            }
        }

        template<typename... Blocks>
        __forceinline void storeAccumulatedImpulses(ConstraintLayout<Blocks...>, Constraint1DReader& constraints) {
            (storeAccumulatedImpulses<Blocks>(constraints), ...);
        }

        template<typename Block>
        __forceinline static void writeConstraints(const Data& data, const Constraint1DDescriptor* rows, int& row, Constraint1DWriter& constraints) {
            if constexpr (Block::gate != nullptr) if (!(data.*Block::gate)) return;
            for (int i = 0; i < Block::count; ++i) {
                const Constraint1DDescriptor& constraintRow = rows[row++];
                auto constraint = constraints.next<Block::flags>();
                constraint
                    .setLinear0(constraintRow.linear0)
                    .setLinear1(constraintRow.linear1)
                    .setAngular0(constraintRow.angular0)
                    .setAngular1(constraintRow.angular1)
                    .setC(constraintRow.geometricError)
                    .setTargetVelocity(constraintRow.targetVelocity);
                if constexpr (Block::flags & LIMITED) constraint.setMin(constraintRow.minForce).setMax(constraintRow.maxForce);
                if constexpr (Block::flags & SOFT) constraint.setStiffness(constraintRow.stiffness).setDamping(constraintRow.damping);
            }
        }

        template<typename... Blocks>
        __forceinline static void writeConstraints(ConstraintLayout<Blocks...>, const Data& data, const Constraint1DDescriptor* rows, Constraint1DWriter& constraints) {
            int row = 0;
            (writeConstraints<Blocks>(data, rows, row, constraints), ...);
        }

    public:
        using Joint::Joint;
        JointSolverDesc getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) override;
        void storeAccumulatedImpulses(Constraint1DReader &constraints) override;
    };

    template<typename Impl, typename Layout, typename Cache, typename Data>
    JointSolverDesc JointImpl<Impl, Layout, Cache, Data>::getSolverDesc(entt::registry &registry, Constraint1DLayout &constraintLayout) {
        static_cast<Impl*>(this)->prepare(registry);
        createConstraints(Layout{}, constraintLayout);
        return JointSolverDesc{ &data, makeFinalConstraints };
    }

    template<typename Impl, typename Layout, typename Cache, typename Data>
    void JointImpl<Impl, Layout, Cache, Data>::storeAccumulatedImpulses(Constraint1DReader &constraints) {
        storeAccumulatedImpulses(Layout{}, constraints);
    }

    template<int numAngularRows>
    inline float computeInvEffMassEntry(const Constraint1DDescriptor* constraintRows, int i, int j) {
        float entry = glm::dot(constraintRows[i].angular0, constraintRows[j].angular0)
            + glm::dot(constraintRows[i].angular1, constraintRows[j].angular1);
        if (i >= numAngularRows && j >= numAngularRows) {
            entry += glm::dot(constraintRows[i].linear0, constraintRows[j].linear0)
                + glm::dot(constraintRows[i].linear1, constraintRows[j].linear1);
        }
        return entry;
    }

    template<int numRows, int numAngularRows>
    void LDLtFactorize(const Constraint1DDescriptor* constraintRows, float L[][numRows], float* D) {
        // LDLt factorization of JM^-1J^T
        for (int i = 0; i < numRows; ++i) {
            D[i] = computeInvEffMassEntry<numAngularRows>(constraintRows, i, i);
            for (int j = 0; j < i; ++j) {
                D[i] -= L[j][i] * L[j][i] * D[j];
            }
            for (int j = i + 1; j < numRows; ++j) {
                L[i][j] = computeInvEffMassEntry<numAngularRows>(constraintRows, j, i);
                for (int k = 0; k < i; ++k) {
                    L[i][j] -= D[k] * L[k][i] * L[k][j];
                }
                L[i][j] /= D[i] + 1e-8f;
            }
        }
    }

    template<int numRows, int numAngularRows>
    void orthogonalize(Constraint1DDescriptor* constraintRows, float L[][numRows]) {
        // solve LJ' = J by forward substitution
        // J'M^-1J'^T will be a diagonal matrix, making gauss seidel for these constraints be identical to a block solve
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < i; ++j) {
                if (j >= numAngularRows) {
                    constraintRows[i].linear0 -= L[j][i] * constraintRows[j].linear0;
                    constraintRows[i].linear1 -= L[j][i] * constraintRows[j].linear1;
                }
                constraintRows[i].angular0 -= L[j][i] * constraintRows[j].angular0;
                constraintRows[i].angular1 -= L[j][i] * constraintRows[j].angular1;
                constraintRows[i].geometricError -= L[j][i] * constraintRows[j].geometricError;
            }
        }
    }

    template<typename Impl, typename Layout, typename Cache, typename Data>
    void JointImpl<Impl, Layout, Cache, Data>::makeFinalConstraints(const JointWorldSpaceData& worldSpaceData, void* additionalData, const Constraint1DWriterContext& context, Constraint1DWriter& constraints) {
        Constraint1DDescriptor constraintRows[Layout::count];
        Impl::makeConstraints(worldSpaceData, additionalData, constraintRows);

        // apply transform and mass scale
        for (int i = 0; i < Layout::count; ++i) {
            constraintRows[i].linear0 *= context.massData0->sqrtInvMass;
            constraintRows[i].linear1 *= context.massData1->sqrtInvMass;
            constraintRows[i].angular0 = multiplyTranspose(*context.r0, constraintRows[i].angular0) * context.massData0->sqrtInvInertia;
            constraintRows[i].angular1 = multiplyTranspose(*context.r1, constraintRows[i].angular1) * context.massData1->sqrtInvInertia;
        }

        constexpr int n = Layout::hardEqualityCount;
        if constexpr (n > 1) {
            float L[n][n]; // column major lower triangular
            float D[n]; // diagonal

            constexpr int numAngular = Layout::hardEqualityAngularCount;
            LDLtFactorize<n, numAngular>(constraintRows, L, D);
            orthogonalize<n, numAngular>(constraintRows, L);
        }

        writeConstraints(Layout{}, *static_cast<const Data*>(additionalData), constraintRows, constraints);
    }

#define PHYSECS_DECLARE_JOINT_IMPL(joint) \
    extern template class JointImpl<joint, joint##Def::Layout, joint##Def::Cache, joint##Def::Data>

#define PHYSECS_DEFINE_JOINT_IMPL(joint) \
    template class JointImpl<joint, joint##Def::Layout, joint##Def::Cache, joint##Def::Data>
}
