/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
GDDP<STATE_DIM, INPUT_DIM>::GDDP(
		const GDDP_Settings& gddpSettings /*= GDDP_Settings()*/)
	: gddpSettings_(gddpSettings)
{
	bvpSensitivityEquationsPtrStock_.clear();
	bvpSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
	bvpSensitivityIntegratorsPtrStock_.clear();
	bvpSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

	bvpSensitivityErrorEquationsPtrStock_.clear();
	bvpSensitivityErrorEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
	bvpSensitivityErrorIntegratorsPtrStock_.clear();
	bvpSensitivityErrorIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

	rolloutSensitivityEquationsPtrStock_.clear();
	rolloutSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
	rolloutSensitivityIntegratorsPtrStock_.clear();
	rolloutSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

	riccatiSensitivityEquationsPtrStock_.clear();
	riccatiSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
	riccatiSensitivityIntegratorsPtrStock_.clear();
	riccatiSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

	for (size_t i=0; i<gddpSettings_.nThreads_; i++)  {

		typedef Eigen::aligned_allocator<bvp_sensitivity_equations_t> bvp_sensitivity_equations_alloc_t;
		bvpSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<bvp_sensitivity_equations_t, bvp_sensitivity_equations_alloc_t>(
						bvp_sensitivity_equations_alloc_t()) ) );

		typedef Eigen::aligned_allocator<bvp_sensitivity_error_equations_t> bvp_sensitivity_error_equations_alloc_t;
		bvpSensitivityErrorEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<bvp_sensitivity_error_equations_t, bvp_sensitivity_error_equations_alloc_t>(
						bvp_sensitivity_error_equations_alloc_t()) ) );

		typedef Eigen::aligned_allocator<rollout_sensitivity_equations_t> rollout_sensitivity_equations_alloc_t;
		rolloutSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<rollout_sensitivity_equations_t, rollout_sensitivity_equations_alloc_t>(
						rollout_sensitivity_equations_alloc_t()) ) );

		typedef Eigen::aligned_allocator<riccati_sensitivity_equations_t> riccati_sensitivity_equations_alloc_t;
		riccatiSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<riccati_sensitivity_equations_t, riccati_sensitivity_equations_alloc_t>(
						riccati_sensitivity_equations_alloc_t()) ) );

		switch(gddpSettings_.RiccatiIntegratorType_) {

		case DIMENSIONS::RiccatiIntegratorType::ODE45 : {
			bvpSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<STATE_DIM>(bvpSensitivityEquationsPtrStock_.back()) );

			bvpSensitivityErrorIntegratorsPtrStock_.emplace_back (
					new ODE45<STATE_DIM>(bvpSensitivityErrorEquationsPtrStock_.back()) );

			rolloutSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<STATE_DIM>(rolloutSensitivityEquationsPtrStock_.back()) );

			riccatiSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<riccati_sensitivity_equations_t::S_DIM_>(
							riccatiSensitivityEquationsPtrStock_.back()) );

			break;
		}
		/* note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
		case DIMENSIONS::RiccatiIntegratorType::ADAMS_BASHFORTH : {
			throw std::runtime_error("This ADAMS_BASHFORTH is not implemented for Riccati Integrator.");
			break;
		}
		case DIMENSIONS::RiccatiIntegratorType::BULIRSCH_STOER : {
			bvpSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<STATE_DIM>(bvpSensitivityEquationsPtrStock_.back()) );

			bvpSensitivityErrorIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<STATE_DIM>(bvpSensitivityErrorEquationsPtrStock_.back()) );

			rolloutSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<STATE_DIM>(rolloutSensitivityEquationsPtrStock_.back()) );

			riccatiSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<riccati_sensitivity_equations_t::S_DIM_>(
							riccatiSensitivityEquationsPtrStock_.back()) );

			break;
		}
		default:
			throw (std::runtime_error("Riccati equations integrator type specified wrongly."));
		}

	}  // end of i loop

	// calculateBVPSensitivityControllerForward & calculateLQSensitivityControllerForward
	BmFuncStock_.resize(gddpSettings_.nThreads_);
	RmInverseFuncStock_.resize(gddpSettings_.nThreads_);
	DmProjectedFuncStock_.resize(gddpSettings_.nThreads_);
	EvDevEventTimesProjectedFuncStock_.resize(gddpSettings_.nThreads_);
	nablaRvFuncStock_.resize(gddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::setupOptimizer(
		const size_t& numPartitions) {

	if (numPartitions==0)
		throw std::runtime_error("The number of partitions cannot be zero!");
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateRolloutCostate(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		state_vector_array2_t& costateTrajectoriesStock,
		scalar_t learningRate /*= 0.0*/ )  {

	costateTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			costateTrajectoriesStock[i].clear();
			continue;
		}

		SmFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->SmTrajectoriesStock_[i]);
		SvFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->SvTrajectoriesStock_[i]);
		SveFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->SveTrajectoriesStock_[i]);
		nominalStateFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->nominalStateTrajectoriesStock_[i]);

		const size_t N = timeTrajectoriesStock[i].size();
		costateTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			const scalar_t& t = timeTrajectoriesStock[i][k];

			state_matrix_t Sm;
			const auto indexAlpha = SmFunc_.interpolate(t, Sm);
			state_vector_t Sv;
			SvFunc_.interpolate(indexAlpha,  Sv);
			state_vector_t Sve;
			SveFunc_.interpolate(indexAlpha,  Sve);

			state_vector_t nominalState;
			nominalStateFunc_.interpolate(t, nominalState);

			costateTrajectoriesStock[i][k] = Sve + Sv
					+ learningRate * Sm * (stateTrajectoriesStock[i][k]-nominalState);

		}  // end of k loop
	}  // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateRolloutCostate(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& costateTrajectoriesStock)  {

	costateTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			costateTrajectoriesStock[i].clear();
			continue;
		}

		SvFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->SvTrajectoriesStock_[i]);
		SveFunc_.setData(&dcPtr_->SsTimeTrajectoriesStock_[i], &dcPtr_->SveTrajectoriesStock_[i]);

		const size_t N = timeTrajectoriesStock[i].size();
		costateTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			const scalar_t& t = timeTrajectoriesStock[i][k];

			state_vector_t Sv;
			auto indexAlpha = SvFunc_.interpolate(t, Sv);
			state_vector_t Sve;
			SveFunc_.interpolate(indexAlpha,  Sve);

			costateTrajectoriesStock[i][k] = Sve + Sv;

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateInputConstraintLagrangian(
		lagrange_array_t& lagrangeMultiplierFunctionsStock,
		scalar_t learningRate /*= 0.0*/) {

	// functions for controller and Lagrange multiplier
	EigenLinearInterpolation<state_vector_t> xFunc;
	EigenLinearInterpolation<state_input_matrix_t> BmFunc;
	EigenLinearInterpolation<input_state_matrix_t> PmFunc;
	EigenLinearInterpolation<input_vector_t> RvFunc;
	EigenLinearInterpolation<input_matrix_t> RmFunc;
	EigenLinearInterpolation<input_vector_t> EvProjectedFunc;
	EigenLinearInterpolation<input_state_matrix_t> CmProjectedFunc;
	EigenLinearInterpolation<input_constraint1_matrix_t> DmDagerFunc;

	lagrangeMultiplierFunctionsStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			lagrangeMultiplierFunctionsStock[i].clear();
			continue;
		}

		xFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->nominalStateTrajectoriesStock_[i]);
		BmFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->BmTrajectoriesStock_[i]);
		PmFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->PmTrajectoriesStock_[i]);
		RvFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->RvTrajectoriesStock_[i]);
		RmFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->RmTrajectoriesStock_[i]);
		EvProjectedFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->EvProjectedTrajectoriesStock_[i]);
		CmProjectedFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->CmProjectedTrajectoriesStock_[i]);
		DmDagerFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->DmDagerTrajectoriesStock_[i]);

		const size_t N = dcPtr_->SsTimeTrajectoriesStock_[i].size();

		lagrangeMultiplierFunctionsStock[i].time_ = dcPtr_->SsTimeTrajectoriesStock_[i];
		lagrangeMultiplierFunctionsStock[i].k_.resize(N);
		lagrangeMultiplierFunctionsStock[i].uff_.resize(N);
		lagrangeMultiplierFunctionsStock[i].deltaUff_.resize(N);

		for (size_t k=0; k<N; k++) {

			const scalar_t& time = dcPtr_->SsTimeTrajectoriesStock_[i][k];

			state_vector_t nominalState;
			const auto indexAlpha = xFunc.interpolate(time, nominalState);
			state_input_matrix_t Bm;
			BmFunc.interpolate(indexAlpha, Bm);
			input_state_matrix_t Pm;
			PmFunc.interpolate(indexAlpha, Pm);
			input_vector_t Rv;
			RvFunc.interpolate(indexAlpha, Rv);
			input_vector_t EvProjected;
			EvProjectedFunc.interpolate(indexAlpha, EvProjected);
			input_state_matrix_t CmProjected;
			CmProjectedFunc.interpolate(indexAlpha, CmProjected);
			input_matrix_t Rm;
			RmFunc.interpolate(indexAlpha, Rm);
			input_constraint1_matrix_t DmDager;
			DmDagerFunc.interpolate(indexAlpha, DmDager);

			const auto greatestLessTimeStampIndex = indexAlpha.first;
			const size_t& nc1 = dcPtr_->nc1TrajectoriesStock_[i][greatestLessTimeStampIndex];

			const state_matrix_t& Sm  = dcPtr_->SmTrajectoriesStock_[i][k];
			const state_vector_t& Sv  = dcPtr_->SvTrajectoriesStock_[i][k];
			const state_vector_t& Sve = dcPtr_->SveTrajectoriesStock_[i][k];

			dynamic_input_matrix_t DmDagerTransRm = DmDager.leftCols(nc1).transpose() * Rm;

			constraint1_state_matrix_t& K  = lagrangeMultiplierFunctionsStock[i].k_[k];
			constraint1_vector_t& uff      = lagrangeMultiplierFunctionsStock[i].uff_[k];
			constraint1_vector_t& deltaUff = lagrangeMultiplierFunctionsStock[i].deltaUff_[k];

			K.topRows(nc1) = learningRate * ( DmDagerTransRm*CmProjected
					- DmDager.leftCols(nc1).transpose()*(Pm + Bm.transpose()*Sm) );
			K.bottomRows(DIMENSIONS::MAX_CONSTRAINT1_DIM_-nc1).setZero();

			uff.head(nc1) = DmDagerTransRm*EvProjected
					- DmDager.leftCols(nc1).transpose()*(Rv + Bm.transpose()*(Sv+Sve))
					- K.topRows(nc1)*nominalState;
			uff.tail(DIMENSIONS::MAX_CONSTRAINT1_DIM_-nc1).setZero();

			deltaUff.setZero();

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateRolloutLagrangeMultiplier(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const lagrange_array_t& lagrangeMultiplierFunctionsStock,
		constraint1_vector_array2_t& lagrangeTrajectoriesStock)  {

	EigenLinearInterpolation<constraint1_vector_t> vffFunc;
	EigenLinearInterpolation<constraint1_state_matrix_t> vfbFunc;

	lagrangeTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			lagrangeTrajectoriesStock[i].clear();
			continue;
		}

		vffFunc.setData(&lagrangeMultiplierFunctionsStock[i].time_, &lagrangeMultiplierFunctionsStock[i].uff_);
		vfbFunc.setData(&lagrangeMultiplierFunctionsStock[i].time_, &lagrangeMultiplierFunctionsStock[i].k_);

		size_t N = timeTrajectoriesStock[i].size();
		lagrangeTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			constraint1_vector_t vff;
			const auto indexAlpha = vffFunc.interpolate(timeTrajectoriesStock[i][k], vff);

			constraint1_state_matrix_t vfb;
			vfbFunc.interpolate(indexAlpha, vfb);

			lagrangeTrajectoriesStock[i][k] = vff + vfb*stateTrajectoriesStock[i][k];

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateNominalRolloutLagrangeMultiplier(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		constraint1_vector_array2_t& lagrangeTrajectoriesStock)  {

	lagrangeTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			lagrangeTrajectoriesStock[i].clear();
			continue;
		}

		size_t N = timeTrajectoriesStock[i].size();
		lagrangeTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			const size_t& nc1 = dcPtr_->nc1TrajectoriesStock_[i][k];
			const state_input_matrix_t& Bm = dcPtr_->BmTrajectoriesStock_[i][k];
			const input_vector_t& Rv = dcPtr_->RvTrajectoriesStock_[i][k];
			const input_matrix_t& Rm = dcPtr_->RmTrajectoriesStock_[i][k];
			const input_vector_t& EvProjected = dcPtr_->EvProjectedTrajectoriesStock_[i][k];
			const input_constraint1_matrix_t& DmDager = dcPtr_->DmDagerTrajectoriesStock_[i][k];
			const state_vector_t& costate = nominalCostateTrajectoriesStock_[i][k];

			lagrangeTrajectoriesStock[i][k].head(nc1) = DmDager.leftCols(nc1).transpose() * (
					Rm*EvProjected - Rv - Bm.transpose()*costate );
			lagrangeTrajectoriesStock[i][k].tail(DIMENSIONS::MAX_CONSTRAINT1_DIM_-nc1).setZero();

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t GDDP<STATE_DIM, INPUT_DIM>::findActiveSubsystemIndex(
		const scalar_array_t& eventTimes,
		const scalar_t& time,
		bool ceilingFunction /*= true*/) const {

	scalar_array_t partitioningTimes(eventTimes.size()+2);
	partitioningTimes.front() = std::numeric_limits<scalar_t>::lowest();
	partitioningTimes.back()  = std::numeric_limits<scalar_t>::max();
	for (size_t i=0; i<eventTimes.size(); i++)
		partitioningTimes[i+1] = eventTimes[i];

	int activeSubsystemIndex;
	if (ceilingFunction==true)
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);
	else
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0,
				-OCS2NumericTraits<scalar_t>::weakEpsilon());

	return (size_t)activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t GDDP<STATE_DIM, INPUT_DIM>::findActivePartitionIndex(
		const scalar_array_t& partitioningTimes,
		const scalar_t& time,
		bool ceilingFunction /*= true*/) const {

	int activeSubsystemIndex;
	if (ceilingFunction==true)
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);
	else
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0,
				-OCS2NumericTraits<scalar_t>::weakEpsilon());

	if (activeSubsystemIndex < 0) {
		std::string mesg = "Given time is less than the start time (i.e. givenTime < partitioningTimes.front()): "
				+ std::to_string(time) + " < " + std::to_string(partitioningTimes.front());
		throw std::runtime_error(mesg);
	}

	if (activeSubsystemIndex == partitioningTimes.size()-1) {
		std::string mesg = "Given time is greater than the final time (i.e. partitioningTimes.back() < givenTime): "
				+ std::to_string(partitioningTimes.back()) + " < " + std::to_string(time);
		throw std::runtime_error(mesg);
	}

	return (size_t)activeSubsystemIndex;
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::computeEquivalentSystemMultiplier(
		const size_t& eventTimeIndex,
		const size_t& activeSubsystem,
		scalar_t& multiplier) const {

	scalar_t timePeriod;

	if (activeSubsystem == eventTimeIndex+1) {
		if (activeSubsystem == eventTimes_.size()) {
			if (dcPtr_->finalTime_<eventTimes_[eventTimeIndex])
				throw std::runtime_error("Final time is smaller than the last triggered event time.");
			else
				timePeriod = dcPtr_->finalTime_ - eventTimes_[eventTimeIndex];
		} else {
			timePeriod = eventTimes_[eventTimeIndex+1] - eventTimes_[eventTimeIndex];
		}

		multiplier = -1.0 / timePeriod;

	} else if (activeSubsystem == eventTimeIndex) {
		if (activeSubsystem == 0) {
			if (dcPtr_->initTime_>eventTimes_[eventTimeIndex])
				throw std::runtime_error("Initial time is greater than the last triggered event time.");
			else
				timePeriod = eventTimes_[eventTimeIndex] - dcPtr_->initTime_;
		} else {
			timePeriod = eventTimes_[eventTimeIndex] - eventTimes_[eventTimeIndex-1];
		}

		multiplier = 1.0 / timePeriod;

	} else {
		timePeriod = 1.0;
		multiplier = 0.0;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::getRolloutSensitivity2EventTime(
		const size_t& eventTimeIndex,
		std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		state_matrix_array2_t& sensitivityStateTrajectoriesStock,
		input_matrix_array2_t& sensitivityInputTrajectoriesStock)  {

	if (eventTimeIndex+1 > numEventTimes_)
		throw std::runtime_error("The requested event index is out of bound.");

	sensitivityTimeTrajectoriesStock  = dcPtr_->nominalTimeTrajectoriesStock_;
	sensitivityStateTrajectoriesStock = sensitivityStateTrajectoriesStockSet_[eventTimeIndex];
	sensitivityInputTrajectoriesStock = sensitivityInputTrajectoriesStockSet_[eventTimeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
GDDP_Settings& GDDP<STATE_DIM, INPUT_DIM>::settings() {

	return gddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
void GDDP<STATE_DIM, INPUT_DIM>::getCostFuntionDerivative(
		Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {

	// refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
	const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename GDDP<STATE_DIM, INPUT_DIM>::scalar_array_t&
	GDDP<STATE_DIM, INPUT_DIM>::eventTimes() const {

	return eventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::propagateRolloutSensitivity(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const linear_controller_array_t& controllersStock,
		const input_vector_array2_t& LvTrajectoriesStock,
		const std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& sensitivityStateTrajectoriesStock,
		input_vector_array2_t& sensitivityInputTrajectoriesStock)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	sensitivityStateTrajectoriesStock.resize(numPartitions_);
	sensitivityInputTrajectoriesStock.resize(numPartitions_);

	// Initial state sensitivity (which is zero)
	state_vector_t nabla_xInit = state_vector_t::Zero();

	for (size_t i=0; i<numPartitions_; i++) {

		// skip inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			sensitivityStateTrajectoriesStock[i].clear();
			sensitivityInputTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N  = sensitivityTimeTrajectoriesStock[i].size();
		const size_t NE = eventsPastTheEndIndecesStock[i].size();

		// set data for rollout sensitivity equation
		rolloutSensitivityEquationsPtrStock_[workerIndex]->reset();
		rolloutSensitivityEquationsPtrStock_[workerIndex]->setData(
				&dcPtr_->nominalTimeTrajectoriesStock_[i],
				&dcPtr_->AmTrajectoriesStock_[i], &dcPtr_->BmTrajectoriesStock_[i],
				&dcPtr_->nominalFlowMapTrajectoriesStock_[i],
				&controllersStock[i].timeStamp_,
				&LvTrajectoriesStock[i], &controllersStock[i].gainArray_);

		// max number of steps of integration
		const size_t maxNumSteps = gddpSettings_.maxNumStepsPerSecond_ *
				std::max(1.0, sensitivityTimeTrajectoriesStock[i].back()-sensitivityTimeTrajectoriesStock[i].front());

		// resizing
		sensitivityStateTrajectoriesStock[i].clear();
		sensitivityStateTrajectoriesStock[i].reserve(N);
		sensitivityInputTrajectoriesStock[i].clear();
		sensitivityInputTrajectoriesStock[i].reserve(N);

		// integrating
		size_t k_u = 0;
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = (j==0) ? sensitivityTimeTrajectoriesStock[i].begin()
					: sensitivityTimeTrajectoriesStock[i].begin() + eventsPastTheEndIndecesStock[i][j-1];
			endTimeItr = (j==NE) ? sensitivityTimeTrajectoriesStock[i].end()
					: sensitivityTimeTrajectoriesStock[i].begin() + eventsPastTheEndIndecesStock[i][j];

			// if it should be integrated
			if (endTimeItr != beginTimeItr) {

				// finding the current active subsystem
				scalar_t midTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				size_t activeSubsystem = findActiveSubsystemIndex(
						eventTimes_, midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				rolloutSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve sensitivity ODE
				rolloutSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						nabla_xInit, beginTimeItr, endTimeItr,
						sensitivityStateTrajectoriesStock[i],
						gddpSettings_.minTimeStep_,
						gddpSettings_.absTolODE_,
						gddpSettings_.relTolODE_,
						maxNumSteps, true);

				// compute input sensitivity
				for ( ; k_u<sensitivityStateTrajectoriesStock[i].size(); k_u++) {
					sensitivityInputTrajectoriesStock[i].emplace_back(
							rolloutSensitivityEquationsPtrStock_[workerIndex]->controllerPtr()->computeInput(
									sensitivityTimeTrajectoriesStock[i][k_u], sensitivityStateTrajectoriesStock[i][k_u]) );
				} // end of k loop
			}

			// compute jump map
			if (j < NE) {
				nabla_xInit =  sensitivityStateTrajectoriesStock[i].back();
			}

		}  // end of j loop

		// reset the initial state
		nabla_xInit = sensitivityStateTrajectoriesStock[i].back();

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::approximateNominalLQPSensitivity2EventTime(
		const state_vector_array2_t& sensitivityStateTrajectoriesStock,
		const input_vector_array2_t& sensitivityInputTrajectoriesStock,
		eigen_scalar_array2_t& nablaqTrajectoriesStock,
		state_vector_array2_t& nablaQvTrajectoriesStock,
		input_vector_array2_t& nablaRvTrajectoriesStock,
		eigen_scalar_array2_t& nablaqFinalStock,
		state_vector_array2_t& nablaQvFinalStock) const {

	// resizing
	nablaqTrajectoriesStock.resize(numPartitions_);
	nablaQvTrajectoriesStock.resize(numPartitions_);
	nablaRvTrajectoriesStock.resize(numPartitions_);
	nablaqFinalStock.resize(numPartitions_);
	nablaQvFinalStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			nablaqTrajectoriesStock[i].clear();
			nablaQvTrajectoriesStock[i].clear();
			nablaRvTrajectoriesStock[i].clear();
			nablaqFinalStock[i].clear();
			nablaQvFinalStock[i].clear();
			continue;
		}

		const size_t N  = dcPtr_->nominalTimeTrajectoriesStock_[i].size();
		const size_t NE = dcPtr_->nominalEventsPastTheEndIndecesStock_[i].size();
		auto eventsPastTheEndItr = dcPtr_->nominalEventsPastTheEndIndecesStock_[i].begin();

		// resizing
		nablaqTrajectoriesStock[i].resize(N);
		nablaQvTrajectoriesStock[i].resize(N);
		nablaRvTrajectoriesStock[i].resize(N);
		nablaqFinalStock[i].resize(NE);
		nablaQvFinalStock[i].resize(NE);

		for (size_t k=0; k<N; k++) {

			const input_matrix_t& Rm = dcPtr_->RmTrajectoriesStock_[i][k];
			const state_vector_t& Qv = dcPtr_->QvTrajectoriesStock_[i][k];
			const state_matrix_t& Qm = dcPtr_->QmTrajectoriesStock_[i][k];
			const input_vector_t& Rv = dcPtr_->RvTrajectoriesStock_[i][k];
			const input_state_matrix_t& Pm = dcPtr_->PmTrajectoriesStock_[i][k];

			nablaqTrajectoriesStock[i][k]  = Qv.transpose()*sensitivityStateTrajectoriesStock[i][k] +
					Rv.transpose()*sensitivityInputTrajectoriesStock[i][k];
			nablaQvTrajectoriesStock[i][k] = Qm*sensitivityStateTrajectoriesStock[i][k] +
					Pm.transpose()*sensitivityInputTrajectoriesStock[i][k];
			nablaRvTrajectoriesStock[i][k] = Pm*sensitivityStateTrajectoriesStock[i][k] +
					Rm*sensitivityInputTrajectoriesStock[i][k];

			// terminal cost sensitivity to switching times
			if (eventsPastTheEndItr!=dcPtr_->nominalEventsPastTheEndIndecesStock_[i].end() && k+1==*eventsPastTheEndItr) {

				const size_t eventIndex = eventsPastTheEndItr - dcPtr_->nominalEventsPastTheEndIndecesStock_[i].begin();
				const size_t timeIndex = *eventsPastTheEndItr - 1;
				const state_vector_t& Qv = dcPtr_->QvFinalStock_[i][eventIndex];
				const state_matrix_t& Qm = dcPtr_->QmFinalStock_[i][eventIndex];

				nablaqFinalStock[i][eventIndex]  = Qv.transpose()*sensitivityStateTrajectoriesStock[i][timeIndex];
				nablaQvFinalStock[i][eventIndex] = Qm*sensitivityStateTrajectoriesStock[i][timeIndex];

				eventsPastTheEndItr++;
			}
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::approximateNominalHeuristicsSensitivity2EventTime(
		const state_vector_t& sensitivityFinalState,
		eigen_scalar_t& nablasHeuristics,
		state_vector_t& nablaSvHeuristics) const {

	nablasHeuristics  = dcPtr_->SvHeuristics_.transpose() * sensitivityFinalState;
	nablaSvHeuristics = dcPtr_->SmHeuristics_ * sensitivityFinalState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::solveSensitivityRiccatiEquations(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const scalar_t& learningRate,
		const eigen_scalar_t& nablasHeuristics,
		const state_vector_t& nablaSvHeuristics,
		const state_matrix_t& nablaSmHeuristics,
		eigen_scalar_array2_t& nablasTrajectoriesStock,
		state_vector_array2_t& nablaSvTrajectoriesStock,
		state_matrix_array2_t& nablaSmTrajectoriesStock)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	typedef typename riccati_sensitivity_equations_t::s_vector_t s_vector_t;
	typedef typename riccati_sensitivity_equations_t::s_vector_array_t s_vector_array_t;

	// Resizing
	nablasTrajectoriesStock.resize(numPartitions_);
	nablaSvTrajectoriesStock.resize(numPartitions_);
	nablaSmTrajectoriesStock.resize(numPartitions_);

	// temporal final value for the last Riccati equations
	s_vector_t SsFinal;
	riccati_sensitivity_equations_t::convert2Vector(
			nablaSmHeuristics,
			nablaSvHeuristics,
			nablasHeuristics,
			SsFinal);
	// output containers which is reverse
	s_vector_array_t allSsTrajectory;

	for (int i=numSubsystems_-1; i>=0; i--) {

		// skip inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			nablasTrajectoriesStock[i].clear();
			nablaSvTrajectoriesStock[i].clear();
			nablaSmTrajectoriesStock[i].clear();
			continue;
		}

		const size_t NS = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
		const size_t NE = dcPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

		// set data for Riccati sensitivity equations
		riccatiSensitivityEquationsPtrStock_[workerIndex]->reset();
		riccatiSensitivityEquationsPtrStock_[workerIndex]->setData(
				learningRate,
				dcPtr_->partitioningTimes_[i],
				dcPtr_->partitioningTimes_[i+1],
				&dcPtr_->SsTimeTrajectoriesStock_[i],
				&dcPtr_->SmTrajectoriesStock_[i],
				&dcPtr_->SvTrajectoriesStock_[i],
				&dcPtr_->nominalTimeTrajectoriesStock_[i],
				&dcPtr_->AmTrajectoriesStock_[i],
				&dcPtr_->BmTrajectoriesStock_[i],
				&dcPtr_->qTrajectoriesStock_[i],
				&dcPtr_->QvTrajectoriesStock_[i],
				&dcPtr_->QmTrajectoriesStock_[i],
				&dcPtr_->RvTrajectoriesStock_[i],
				&dcPtr_->RmInverseTrajectoriesStock_[i],
				&dcPtr_->RmTrajectoriesStock_[i],
				&dcPtr_->PmTrajectoriesStock_[i],
				&nablaqTrajectoriesStockSet_[eventTimeIndex][i],
				&nablaQvTrajectoriesStockSet_[eventTimeIndex][i],
				&nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

		// max number of steps of integration
		const size_t maxNumSteps = gddpSettings_.maxNumStepsPerSecond_ *
				std::max(1.0, dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].back()-dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

		// output containers resizing
		allSsTrajectory.clear();
		allSsTrajectory.reserve(NS);

		// normalized switching times
		size_array_t SsNormalizedSwitchingTimesIndices;
		SsNormalizedSwitchingTimesIndices.reserve(NE+2);
		SsNormalizedSwitchingTimesIndices.push_back( 0 );
		for (size_t k=0; k<NE; k++) {
			const size_t& index = dcPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
			SsNormalizedSwitchingTimesIndices.push_back(index);
		}
		SsNormalizedSwitchingTimesIndices.push_back( NS );

		// integrating the Riccati sensitivity equations
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
			endTimeItr   = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j+1];

			// if the event time does not take place at the end of partition
			if (*beginTimeItr < *(endTimeItr-1)) {

				// finding the current active subsystem
				scalar_t midNormalizedTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				scalar_t midTime = dcPtr_->partitioningTimes_[i+1] - (dcPtr_->partitioningTimes_[i+1]-dcPtr_->partitioningTimes_[i])*midNormalizedTime;
				size_t activeSubsystem = findActiveSubsystemIndex(eventTimes_, midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				riccatiSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve Riccati sensitivity equations
				riccatiSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						SsFinal, beginTimeItr, endTimeItr,
						allSsTrajectory,
						gddpSettings_.minTimeStep_,
						gddpSettings_.absTolODE_,
						gddpSettings_.relTolODE_,
						maxNumSteps, true);

			} else {
				allSsTrajectory.push_back(SsFinal);
			}

			// final value of the next subsystem
			if (j < NE) {
				SsFinal = allSsTrajectory.back();

				s_vector_t SsFinalTemp = s_vector_t::Zero();
				riccati_sensitivity_equations_t::convert2Vector(
						state_matrix_t::Zero(),
						nablaQvFinalStockSet_[eventTimeIndex][i][NE-1-j],
						nablaqFinalStockSet_[eventTimeIndex][i][NE-1-j],
						SsFinalTemp);

				SsFinal += SsFinalTemp;
			}

		}  // end of j loop

		// final value of the next partition
		SsFinal = allSsTrajectory.back();

		// check size
		if (allSsTrajectory.size() != NS)
			throw std::runtime_error("allSsTrajectory size is incorrect.");

		// construct 'nable_Sm', 'nable_Sv', and 'nable_s'
		nablasTrajectoriesStock[i].resize(NS);
		nablaSvTrajectoriesStock[i].resize(NS);
		nablaSmTrajectoriesStock[i].resize(NS);
		for (size_t k=0; k<NS; k++) {
			riccati_sensitivity_equations_t::convert2Matrix(allSsTrajectory[NS-1-k],
					nablaSmTrajectoriesStock[i][k], nablaSvTrajectoriesStock[i][k], nablasTrajectoriesStock[i][k]);
		}  // end of k loop

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::solveSensitivityBVP(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const state_vector_t& MvFinal,
		const state_vector_t& MveFinal,
		state_vector_array2_t& MvTrajectoriesStock,
		state_vector_array2_t& MveTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	MvTrajectoriesStock.resize(numPartitions_);
	MveTrajectoriesStock.resize(numPartitions_);

	// temporal final value for the last Riccati equations
	state_vector_t MvFinalInternal  = MvFinal;
	state_vector_t MveFinalInternal = MveFinal;
	// output containers which is a reverse container
	state_vector_array_t rMvTrajectory;
	state_vector_array_t rMveTrajectory;

	for (int i=numPartitions_-1; i>=0; i--) {

		// skip inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			MvTrajectoriesStock[i].clear();
			MveTrajectoriesStock[i].clear();
			continue;
		}

		// set data for Riccati equations
		bvpSensitivityEquationsPtrStock_[workerIndex]->reset();
		bvpSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
		bvpSensitivityEquationsPtrStock_[workerIndex]->setData(
				dcPtr_->partitioningTimes_[i],
				dcPtr_->partitioningTimes_[i+1],
				&dcPtr_->nominalTimeTrajectoriesStock_[i],
				&dcPtr_->AmTrajectoriesStock_[i],
				&dcPtr_->BmTrajectoriesStock_[i],
				&dcPtr_->CmTrajectoriesStock_[i],
				&dcPtr_->AmConstrainedTrajectoriesStock_[i],
				&dcPtr_->CmProjectedTrajectoriesStock_[i],
				&dcPtr_->QvTrajectoriesStock_[i],
				&dcPtr_->nominalFlowMapTrajectoriesStock_[i],
				&nominalCostateTrajectoriesStock_[i],
				&nominalLagrangianTrajectoriesStock_[i],
				&dcPtr_->optimizedControllersStock_[i].timeStamp_,
				&dcPtr_->optimizedControllersStock_[i].gainArray_,
				&dcPtr_->SmTrajectoriesStock_[i]);

		// set data for Riccati error equations
		bvpSensitivityErrorEquationsPtrStock_[workerIndex]->reset();
		bvpSensitivityErrorEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
		bvpSensitivityErrorEquationsPtrStock_[workerIndex]->setData(
				dcPtr_->partitioningTimes_[i],
				dcPtr_->partitioningTimes_[i+1],
				&dcPtr_->nominalTimeTrajectoriesStock_[i],
				&dcPtr_->BmTrajectoriesStock_[i],
				&dcPtr_->AmConstrainedTrajectoriesStock_[i],
				&dcPtr_->CmProjectedTrajectoriesStock_[i],
				&dcPtr_->PmTrajectoriesStock_[i],
				&dcPtr_->RmTrajectoriesStock_[i],
				&dcPtr_->RmInverseTrajectoriesStock_[i],
				&dcPtr_->RmInvConstrainedCholTrajectoryStock_[i],
				&dcPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i],
				&dcPtr_->SsTimeTrajectoriesStock_[i],
				&dcPtr_->SmTrajectoriesStock_[i]);

		const size_t NS = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
		const size_t NE = dcPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

		// max number of steps of integration
		const size_t maxNumSteps = gddpSettings_.maxNumStepsPerSecond_ *
				std::max(1.0, dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].back()-dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

		// output containers resizing
		rMvTrajectory.clear();
		rMvTrajectory.reserve(NS);
		rMveTrajectory.clear();
		rMveTrajectory.reserve(NS);

		// normalized switching times
		size_array_t SsNormalizedSwitchingTimesIndices;
		SsNormalizedSwitchingTimesIndices.reserve(NE+2);
		SsNormalizedSwitchingTimesIndices.push_back( 0 );
		for (size_t k=0; k<NE; k++) {
			const size_t& index = dcPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
			SsNormalizedSwitchingTimesIndices.push_back(index);
		}
		SsNormalizedSwitchingTimesIndices.push_back( NS );

		// integrating the Riccati equations
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
			endTimeItr   = dcPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j+1];

			// if the event time does not take place at the end of partition
			if (*beginTimeItr < *(endTimeItr-1)) {

				// finding the current active subsystem
				scalar_t midNormalizedTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				scalar_t midTime = dcPtr_->partitioningTimes_[i+1] - (dcPtr_->partitioningTimes_[i+1]-dcPtr_->partitioningTimes_[i])*midNormalizedTime;
				size_t activeSubsystem = findActiveSubsystemIndex(eventTimes_, midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				bvpSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve Riccati equations for Mv
				bvpSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						MvFinalInternal, beginTimeItr, endTimeItr,
						rMvTrajectory,
						gddpSettings_.minTimeStep_,
						gddpSettings_.absTolODE_,
						gddpSettings_.relTolODE_,
						maxNumSteps,
						true);

				// solve Riccati equations for Mve
				bvpSensitivityErrorIntegratorsPtrStock_[workerIndex]->integrate(
						MveFinalInternal, beginTimeItr, endTimeItr,
						rMveTrajectory,
						gddpSettings_.minTimeStep_,
						gddpSettings_.absTolODE_,
						gddpSettings_.relTolODE_,
						maxNumSteps,
						true);

			} else {
				rMvTrajectory.push_back(MvFinalInternal);
				rMveTrajectory.push_back(MveFinalInternal);
			}

			// final value of the next subsystem
			if (j < NE) {
				MvFinalInternal  = rMvTrajectory.back();
//				MvFinalInternal += dcPtr_->QvFinalStock_[i][NE-1-j];
				MveFinalInternal = rMveTrajectory.back();
			}

		}  // end of j loop

		// final value of the next partition
		MvFinalInternal  = rMvTrajectory.back();
		MveFinalInternal = rMveTrajectory.back();

		// check sizes
		if (rMvTrajectory.size() != NS)
			throw std::runtime_error("MvTrajectory size is incorrect.");
		if (rMveTrajectory.size() != NS)
			throw std::runtime_error("MveTrajectory size is incorrect.");

		// constructing 'Mv' and 'Mve'
		MvTrajectoriesStock[i].resize(NS);
		std::reverse_copy(rMvTrajectory.begin(), rMvTrajectory.end(), MvTrajectoriesStock[i].begin());
		MveTrajectoriesStock[i].resize(NS);
		std::reverse_copy(rMveTrajectory.begin(), rMveTrajectory.end(), MveTrajectoriesStock[i].begin());

		// testing the numerical stability of the Riccati equations
		if (gddpSettings_.checkNumericalStability_)
			for (int k=NS-1; k>=0; k--) {
				try {
					if (!MvTrajectoriesStock[i][k].allFinite()) throw std::runtime_error("Mv is unstable.");
					if (!MveTrajectoriesStock[i][k].allFinite()) throw std::runtime_error("Mve is unstable.");

				} catch(const std::exception& error) {
					std::cerr << "what(): " << error.what() << " at time " << dcPtr_->SsTimeTrajectoriesStock_[i][k] << " [sec]." << std::endl;
					for (int kp=k; kp<k+10; kp++)  {
						if (kp >= NS) continue;
						std::cerr << "Mv[" << dcPtr_->SsTimeTrajectoriesStock_[i][kp] <<
								"]:\t"<< MvTrajectoriesStock[i][kp].transpose().norm() << std::endl;
						std::cerr << "Mve[" << dcPtr_->SsTimeTrajectoriesStock_[i][kp] <<
								"]:\t"<< MveTrajectoriesStock[i][kp].transpose().norm() << std::endl;
					}
					throw;
				}
			}

	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateLQSensitivityControllerForward(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& nablaSvTrajectoriesStock,
		input_vector_array2_t& nablaLvTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	nablaLvTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numSubsystems_; i++) {

		// skip inactive partitions
		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			nablaLvTrajectoriesStock[i].clear();
			continue;
		}

		// set data
		BmFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->BmTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->RmInverseTrajectoriesStock_[i]);
		nablaRvFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

		// resizing
		const size_t N = nablaSvTrajectoriesStock[i].size();
		nablaLvTrajectoriesStock[i].resize(N);

		for (size_t k=0; k<N; k++) {

			// time
			const scalar_t& t = timeTrajectoriesStock[i][k];

			// Bm
			state_input_matrix_t Bm;
			auto indexAlpha = BmFuncStock_[workerIndex].interpolate(t, Bm);
			// RmInverse
			input_matrix_t RmInverse;
			RmInverseFuncStock_[workerIndex].interpolate(indexAlpha,  RmInverse);
			// nablaRv
			input_vector_t nablaRv;
			nablaRvFuncStock_[workerIndex].interpolate(indexAlpha,  nablaRv);

			nablaLvTrajectoriesStock[i][k] = -RmInverse * (nablaRv + Bm.transpose()*nablaSvTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateBVPSensitivityControllerForward(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& MvTrajectoriesStock,
		const state_vector_array2_t& MveTrajectoriesStock,
		input_vector_array2_t& LvTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	LvTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		if (i<dcPtr_->initActivePartition_ || i>dcPtr_->finalActivePartition_) {
			LvTrajectoriesStock[i].clear();
			continue;
		}

		// set data
		BmFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->BmTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->RmInverseTrajectoriesStock_[i]);
		DmProjectedFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->DmProjectedTrajectoriesStock_[i]);
		EvDevEventTimesProjectedFuncStock_[workerIndex].setData(&dcPtr_->nominalTimeTrajectoriesStock_[i], &dcPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i]);

		const size_t N = timeTrajectoriesStock[i].size();
		LvTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			// time
			const scalar_t& t = timeTrajectoriesStock[i][k];
			// Bm
			state_input_matrix_t Bm;
			auto indexAlpha = BmFuncStock_[workerIndex].interpolate(t, Bm);
			// RmInverse
			input_matrix_t RmInverse;
			RmInverseFuncStock_[workerIndex].interpolate(indexAlpha,  RmInverse);
			// DmProjected
			input_matrix_t DmProjected;
			DmProjectedFuncStock_[workerIndex].interpolate(indexAlpha,  DmProjected);
			// EvDevEventTimesProjected
			input_vector_t EvDevEventTimeProjected;
			EvDevEventTimesProjectedFuncStock_[workerIndex].interpolate(indexAlpha,  EvDevEventTimeProjected);

			LvTrajectoriesStock[i][k] =
					-(input_matrix_t::Identity()-DmProjected) * RmInverse * Bm.transpose() * (
							MvTrajectoriesStock[i][k]+MveTrajectoriesStock[i][k]) - EvDevEventTimeProjected;
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::getValueFuntionDerivative(
		const size_t& eventTimeIndex,
		const scalar_t& time,
		const state_vector_t& state,
		scalar_t& valueFunctionDerivative)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	EigenLinearInterpolation<state_vector_t> nominalStateFunc;
	EigenLinearInterpolation<eigen_scalar_t> nablasFunc;
	EigenLinearInterpolation<state_vector_t> nablaSvFunc;
	EigenLinearInterpolation<state_matrix_t> nablaSmFunc;

	size_t activePartition = findActivePartitionIndex(dcPtr_->partitioningTimes_, time);

	state_vector_t nominalState;
	state_vector_t deltsState;
	eigen_scalar_t nablas;
	state_vector_t nablaSv;
	state_matrix_t nablaSm;;

	nominalStateFunc.setData(&dcPtr_->nominalTimeTrajectoriesStock_[activePartition], &dcPtr_->nominalStateTrajectoriesStock_[activePartition]);
	nominalStateFunc.interpolate(time, nominalState);
	deltsState = state - nominalState;

	nablasFunc.setData(&dcPtr_->SsTimeTrajectoriesStock_[activePartition], &nablasTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	const auto indexAlpha = nablasFunc.interpolate(time, nablas);

	nablaSvFunc.setData(&dcPtr_->SsTimeTrajectoriesStock_[activePartition], &nablaSvTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	nablaSvFunc.interpolate(indexAlpha, nablaSv);

	nablaSmFunc.setData(&dcPtr_->SsTimeTrajectoriesStock_[activePartition], &nablaSmTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	nablaSmFunc.interpolate(indexAlpha, nablaSm);

	valueFunctionDerivative = nablas(0) +
			deltsState.dot(nablaSv) + 0.5*deltsState.dot(nablaSm*deltsState);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateCostDerivative(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const state_vector_array2_t& sensitivityStateTrajectoriesStock,
		const input_vector_array2_t& sensitivityInputTrajectoriesStock,
		scalar_t& costDerivative) const {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	costDerivative = 0.0;
	scalar_t prevIntermediatecostDev = 0.0;
	scalar_t currIntermediatecostDev = 0.0;
	size_t beginIndex, endIndex;

	for (size_t i=dcPtr_->initActivePartition_; i<=dcPtr_->finalActivePartition_; i++) {

		const size_t N  = dcPtr_->nominalTimeTrajectoriesStock_[i].size();
		const size_t NE = dcPtr_->nominalEventsPastTheEndIndecesStock_[i].size();

		for (size_t j=0; j<=NE; j++) {

			beginIndex = (j==0)  ? 0 : dcPtr_->nominalEventsPastTheEndIndecesStock_[i][j-1];
			endIndex   = (j==NE) ? N : dcPtr_->nominalEventsPastTheEndIndecesStock_[i][j];

			// integrates the intermediate cost sensitivity using the trapezoidal approximation method
			if (beginIndex != endIndex) {

				// finding the current active subsystem
				scalar_t midTime = 0.5 * ( dcPtr_->nominalTimeTrajectoriesStock_[i][beginIndex] +
						dcPtr_->nominalTimeTrajectoriesStock_[i][endIndex-1]);
				size_t activeSubsystem = findActiveSubsystemIndex(eventTimes_, midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);

				for (size_t k=beginIndex; k<endIndex; k++) {

					if (k>beginIndex)
						prevIntermediatecostDev = currIntermediatecostDev;

					currIntermediatecostDev =
							multiplier * dcPtr_->qTrajectoriesStock_[i][k](0) +
							sensitivityStateTrajectoriesStock[i][k].dot(dcPtr_->QvTrajectoriesStock_[i][k]) +
							sensitivityInputTrajectoriesStock[i][k].dot(dcPtr_->RvTrajectoriesStock_[i][k]);

					if (k>beginIndex) {
						costDerivative += 0.5*(dcPtr_->nominalTimeTrajectoriesStock_[i][k]-dcPtr_->nominalTimeTrajectoriesStock_[i][k-1]) *
								(currIntermediatecostDev+prevIntermediatecostDev);
					}
				}  // end of k loop
			}

			// terminal cost sensitivity at switching times
			if (j < NE) {
				costDerivative += sensitivityStateTrajectoriesStock[i].back().dot(dcPtr_->QvFinalStock_[i][j]);
			}
		}  // end of j loop

	}  // end of i loop

	// add the Heuristics function sensitivity at the final time
	costDerivative += sensitivityStateTrajectoriesStock[dcPtr_->finalActivePartition_].back().dot(dcPtr_->SvHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::runLQBasedMethod()  {

	const size_t maxNumIteration = 3;

	// resizing
	nablaLvTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
	nablaqTrajectoriesStockSet_.resize(numEventTimes_);
	nablaQvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaRvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaqFinalStockSet_.resize(numEventTimes_);
	nablaQvFinalStockSet_.resize(numEventTimes_);
	nablasHeuristics_.resize(numEventTimes_);
	nablaSvHeuristics_.resize(numEventTimes_);
	nablasTrajectoriesStockSet_.resize(numEventTimes_);
	nablaSvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaSmTrajectoriesStockSet_.resize(numEventTimes_);
	nominalCostFuntionDerivative_.resize(numEventTimes_);

	size_t iteration = 0;
	while (iteration++ < maxNumIteration) {

		// for each active event time
		for (size_t index=0; index<numEventTimes_; index++) {

			if (activeEventTimeBeginIndex_<=index && index<activeEventTimeEndIndex_) {

				// for the first iteration set Lv to zero
				if (iteration == 1) {
					nablaLvTrajectoriesStockSet_[index].resize(numPartitions_);
					for (size_t i=dcPtr_->initActivePartition_; i<=dcPtr_->finalActivePartition_; i++)
						nablaLvTrajectoriesStockSet_[index][i] = input_vector_array_t(
								dcPtr_->optimizedControllersStock_[i].timeStamp_.size(), input_vector_t::Zero());
				}

				const size_t workerIndex = 0;

				// calculate rollout sensitivity to event times
				propagateRolloutSensitivity(workerIndex, index,
						dcPtr_->optimizedControllersStock_,
						nablaLvTrajectoriesStockSet_[index],
						dcPtr_->nominalTimeTrajectoriesStock_,
						dcPtr_->nominalEventsPastTheEndIndecesStock_,
						sensitivityStateTrajectoriesStockSet_[index],
						sensitivityInputTrajectoriesStockSet_[index]);

				// approximate the nominal LQ sensitivity to switching times
				approximateNominalLQPSensitivity2EventTime(
						sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index],
						nablaqTrajectoriesStockSet_[index],
						nablaQvTrajectoriesStockSet_[index], nablaRvTrajectoriesStockSet_[index],
						nablaqFinalStockSet_[index], nablaQvFinalStockSet_[index]);

				// approximate Heuristics
				approximateNominalHeuristicsSensitivity2EventTime(
						sensitivityStateTrajectoriesStockSet_[index][dcPtr_->finalActivePartition_].back(),
						nablasHeuristics_[index], nablaSvHeuristics_[index]);

				// solve Riccati equations
				// prevents the changes in the nominal trajectories and just update the gains
				const scalar_t learningRateStar = 0.0;
				solveSensitivityRiccatiEquations(workerIndex, index,
						learningRateStar,
						nablasHeuristics_[index], nablaSvHeuristics_[index], state_matrix_t::Zero(),
						nablasTrajectoriesStockSet_[index],
						nablaSvTrajectoriesStockSet_[index],
						nablaSmTrajectoriesStockSet_[index]);

				// calculate sensitivity controller feedforward part
				calculateLQSensitivityControllerForward(workerIndex, index,
						dcPtr_->SsTimeTrajectoriesStock_, nablaSvTrajectoriesStockSet_[index],
						nablaLvTrajectoriesStockSet_[index]);

				// calculate the value function derivatives w.r.t. event times
				getValueFuntionDerivative(index, dcPtr_->initTime_, dcPtr_->initState_,
						nominalCostFuntionDerivative_(index));

			} else if (iteration == 1) {
				nablaLvTrajectoriesStockSet_[index].clear();
				sensitivityStateTrajectoriesStockSet_[index].clear();
				sensitivityInputTrajectoriesStockSet_[index].clear();
				nablaqTrajectoriesStockSet_[index].clear();
				nablaQvTrajectoriesStockSet_[index].clear();
				nablaRvTrajectoriesStockSet_[index].clear();
				nablaqFinalStockSet_[index].clear();
				nablaQvFinalStockSet_[index].clear();
				nablasTrajectoriesStockSet_[index].clear();
				nablaSvTrajectoriesStockSet_[index].clear();
				nablaSmTrajectoriesStockSet_[index].clear();
				nominalCostFuntionDerivative_(index) = 0.0;
			}

		}  // end of index loop

	}  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::runSweepingBVPMethod()  {

	// calculate costate
	calculateRolloutCostate(dcPtr_->nominalTimeTrajectoriesStock_,
			nominalCostateTrajectoriesStock_);

	// calculate Lagrangian
	calculateNominalRolloutLagrangeMultiplier(dcPtr_->nominalTimeTrajectoriesStock_,
			nominalLagrangianTrajectoriesStock_);

	// resizing
	MvTrajectoriesStockSet_.resize(numEventTimes_);
	MveTrajectoriesStockSet_.resize(numEventTimes_);
	LvTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
	nominalCostFuntionDerivative_.resize(numEventTimes_);

	// for each active event time
	for (size_t index=0; index<numEventTimes_; index++) {

		if (activeEventTimeBeginIndex_<=index && index<activeEventTimeEndIndex_) {

			const size_t workerIndex = 0;

			// solve BVP to compute 'Mv' and 'Mve'
			solveSensitivityBVP(workerIndex, index,
					state_vector_t::Zero() /*dcPtr_->SvHeuristics_*/,
					state_vector_t::Zero() /*SveHeuristics_*/,
					MvTrajectoriesStockSet_[index],
					MveTrajectoriesStockSet_[index]);

			// calculates sensitivity controller feedforward part, 'Lv'
			calculateBVPSensitivityControllerForward(workerIndex, index,
					dcPtr_->SsTimeTrajectoriesStock_,
					MvTrajectoriesStockSet_[index],
					MveTrajectoriesStockSet_[index],
					LvTrajectoriesStockSet_[index]);

			// calculate rollout sensitivity to event times
			propagateRolloutSensitivity(workerIndex, index,
					dcPtr_->optimizedControllersStock_,
					LvTrajectoriesStockSet_[index],
					dcPtr_->nominalTimeTrajectoriesStock_,
					dcPtr_->nominalEventsPastTheEndIndecesStock_,
					sensitivityStateTrajectoriesStockSet_[index],
					sensitivityInputTrajectoriesStockSet_[index]);

			// calculate the cost function derivatives w.r.t. event times
			calculateCostDerivative(workerIndex, index,
					sensitivityStateTrajectoriesStockSet_[index],
					sensitivityInputTrajectoriesStockSet_[index],
					nominalCostFuntionDerivative_(index));

		} else {
			MvTrajectoriesStockSet_[index].clear();
			MveTrajectoriesStockSet_[index].clear();
			LvTrajectoriesStockSet_[index].clear();
			sensitivityStateTrajectoriesStockSet_[index].clear();
			sensitivityInputTrajectoriesStockSet_[index].clear();
			nominalCostFuntionDerivative_(index) = 0.0;
		}

	} // end of index
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::run(
		const scalar_array_t& eventTimes,
		const slq_data_collector_t* dcPtr)  {

	// event time an number of event and subsystems
	eventTimes_ = eventTimes;
	numEventTimes_ = eventTimes_.size();
	numSubsystems_ = numEventTimes_ + 1;

	// data collector pointer
	dcPtr_ = dcPtr;

	// update sizes if number of partitions has been changed
	if (numPartitions_ != dcPtr_->numPartitions_) {
		numPartitions_ = dcPtr_->numPartitions_;
		setupOptimizer(numPartitions_);
	}

	// find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
	activeEventTimeBeginIndex_ = findActiveSubsystemIndex(eventTimes_, dcPtr_->initTime_);
	activeEventTimeEndIndex_   = findActiveSubsystemIndex(eventTimes_, dcPtr_->finalTime_);

	// display
	if (gddpSettings_.displayInfo_)
		std::cerr << "\n#### Calculating cost function sensitivity ..." << std::endl;

	// use the LQ-based method or Sweeping-BVP method
	if (gddpSettings_.useLQForDerivatives_==true) {
		runLQBasedMethod();
	} else {
		runSweepingBVPMethod();
	}
}


} // namespace ocs2