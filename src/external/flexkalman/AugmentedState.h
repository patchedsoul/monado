/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Internal Includes
#include "FlexibleKalmanBase.h"

// Library/third-party includes
// - none

// Standard includes
#include <type_traits>

namespace flexkalman {

/// State type that consists entirely of references to two independent
/// sub-states.
template <typename StateA, typename StateB> class AugmentedState {
  public:
    using StateTypeA = StateA;
    using StateTypeB = StateB;

    static constexpr size_t DimA = getDimension<StateA>();
    static constexpr size_t DimB = getDimension<StateB>();
    static constexpr size_t Dimension = DimA + DimB;

    using SquareMatrix = types::SquareMatrix<Dimension>;
    using StateVector = types::Vector<Dimension>;

    /// Constructor
    AugmentedState(StateA &a, StateB &b) : a_(a), b_(b) {}

    /// Copy constructor
    AugmentedState(AugmentedState const &other) = default;

    /// Move constructor
    AugmentedState(AugmentedState &&other) : a_(other.a_), b_(other.b_) {}

    /// non-assignable
    AugmentedState &operator=(AugmentedState const &other) = delete;

    /// @name Methods required of State types
    /// @{
    template <typename Derived>
    void setStateVector(Eigen::MatrixBase<Derived> const &state) {
        /// template used here to avoid a temporary
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, Dimension);
        a().setStateVector(state.derived().template head<DimA>());
        b().setStateVector(state.derived().template tail<DimB>());
    }

    StateVector stateVector() const {
        StateVector ret;
        ret << a().stateVector(), b().stateVector();
        return ret;
    }

    SquareMatrix errorCovariance() const {
        SquareMatrix ret = SquareMatrix::Zero();
        ret.template topLeftCorner<DimA, DimA>() = a().errorCovariance();
        ret.template bottomRightCorner<DimB, DimB>() = b().errorCovariance();
        return ret;
    }

    template <typename Derived>
    void setErrorCovariance(Eigen::MatrixBase<Derived> const &P) {
        /// template used here to avoid evaluating elements we'll never
        /// access to a temporary.
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, Dimension, Dimension);
        a().setErrorCovariance(P.template topLeftCorner<DimA, DimA>());
        b().setErrorCovariance(P.template bottomRightCorner<DimB, DimB>());
    }

    void postCorrect() {
        a().postCorrect();
        b().postCorrect();
    }
    /// @}

    /// @name Access to the components of the state
    /// @{
    /// Access the first part of the state
    StateTypeA &a() { return a_; }
    /// Access the first part of the state
    StateTypeA const &a() const { return a_; }

    /// Access the second part of the state
    StateTypeB &b() { return b_; }
    /// Access the second part of the state
    StateTypeB const &b() const { return b_; }
    /// @}

  private:
    StateA &a_;
    StateB &b_;
};
/// Template alias to make removing const from the deduced types less
/// verbose/painful.
template <typename StateA, typename StateB>
using DeducedAugmentedState =
    AugmentedState<typename std::remove_const<StateA>::type,
                   typename std::remove_const<StateB>::type>;

/// Factory function, akin to `std::tie()`, to make an augmented state.
template <typename StateA, typename StateB>
inline DeducedAugmentedState<StateA, StateB> makeAugmentedState(StateA &a,
                                                                StateB &b) {
    return DeducedAugmentedState<StateA, StateB>{a, b};
}

} // namespace flexkalman
