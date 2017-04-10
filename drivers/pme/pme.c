/* 
 * Copyright for portions this project are held by Intel Corporation, 2016 as part of their PME Driver for Arduino project. https://github.com/01org/Intel-Pattern-Matching-Technology
 * Copyright for portions this project are held by General Vision, 2016 as part of their free CurieNeurons Driver for Arduino project. http://www.general-vision.com/download/curieneurons-free-library/
 * All other copyright for this project are held by LAAS-CNRS, 2017.
 * This library is free software. You can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "pme.h"

int pme_begin() {
	PME->forget_ncount = 0;
	PME->nsr = PME_NSR_SR_MODE_MASK;
	for (int i = 0; i < PME_MAX_NEURONS; i++) {
		PME->testcomp = 0;
	}
	PME->nsr = 0;
	if (PME->minif == 2) {
		return (0);
	} else {
		return (1);
	}
}

void pme_forget() {
	PME->forget_ncount = 0;
}

int pme_learn(uint8_t vector[], uint8_t length, uint16_t category) {
	if (length > PME_USER_MAX_VECTOR_SIZE) {
		length = PME_USER_MAX_VECTOR_SIZE;
	}
	for (int i = 0; i < length - 1; i++) {
		PME->comp = vector[i];
	}
	PME->lcomp = vector[length - 1];
	PME->cat = category;
	return (PME->forget_ncount);
}

int pme_read_neurons(neuron_data_Str ptr_neurons_data[]) {
	uint8_t ncount = PME->forget_ncount;
	uint16_t temp_nsr = PME->nsr; // save value to restore NN status upon exit of the function
	PME->nsr |= PME_NSR_SR_MODE_MASK; // Enter Save and Restore mode
	PME->rstchain = 0; // reset to the 0th neuron
	neuron_data_Str * ptr_last_neuron = ptr_neurons_data
			+ ncount;
	for (neuron_data_Str * ptr_current_neuron = ptr_neurons_data;
			ptr_current_neuron < ptr_last_neuron; ptr_current_neuron++) {
		ptr_current_neuron->context = PME->ncr & PME_NCR_CONTEXT_MASK;
		for (uint8_t i = 0; i < PME_USER_MAX_VECTOR_SIZE; i++) {
			ptr_current_neuron->vector[i] = PME->comp;
		}
		ptr_current_neuron->influence = PME->aif;
		ptr_current_neuron->min_influence = PME->minif;
		ptr_current_neuron->category = PME->cat;
	}
	PME->nsr = temp_nsr & ~PME_NSR_SR_MODE_MASK; // set the NN back to its calling status and ensure that we leave Save and Restore Mode
	return (ncount);
}

int pme_write_neurons(const neuron_data_Str ptr_neurons_data[],
		uint8_t number_of_neurons) {
	uint16_t temp_nsr = PME->nsr;
	PME->forget_ncount = 0;
	PME->nsr |= PME_NSR_SR_MODE_MASK;
	PME->rstchain = 0;
	if (number_of_neurons > PME_USER_MAX_VECTOR_SIZE) {
		number_of_neurons = PME_USER_MAX_VECTOR_SIZE;
	}
	const neuron_data_Str * ptr_last_neuron = ptr_neurons_data
			+ number_of_neurons;
	for (const neuron_data_Str * ptr_current_neuron = ptr_neurons_data;
			ptr_current_neuron < ptr_last_neuron; ptr_current_neuron++) {
		PME->ncr = ptr_current_neuron->context & PME_NCR_CONTEXT_MASK;
		for (uint8_t i = 0; i < PME_USER_MAX_VECTOR_SIZE; i++) {
			PME->comp = ptr_current_neuron->vector[i];
		}
		PME->aif = ptr_current_neuron->influence;
		PME->minif = ptr_current_neuron->min_influence;
		PME->cat = ptr_current_neuron->category;
	}
	PME->nsr = temp_nsr & ~PME_NSR_SR_MODE_MASK; // set the NN back to its calling status and ensure that we leave Save and Restore Mode
	return (PME->forget_ncount);
}

int pme_classify(const uint8_t vector[], uint8_t length, uint16_t distance[],
		uint16_t category[], uint8_t neuron_id[], uint8_t max_firing_neurons) {
	uint8_t number_of_firing_neurons = 0;
	if (length > PME_USER_MAX_VECTOR_SIZE) {
		length = PME_USER_MAX_VECTOR_SIZE;
	}
	for (uint8_t i = 0; i < length - 1; i++) {
		PME->comp = vector[i];
	}
	PME->lcomp = vector[length - 1];
	uint16_t dist = PME->idx_dist;
	while ((dist != 0xFFFF) && (number_of_firing_neurons < max_firing_neurons)) {
		distance[number_of_firing_neurons] = dist;
		category[number_of_firing_neurons] = PME->cat;
		neuron_id[number_of_firing_neurons] = PME->nid;
		number_of_firing_neurons++;
		dist = PME->idx_dist;
	}
	return number_of_firing_neurons;
}

void pme_configure(uint8_t global_context, PME_DISTANCE_MODE distance_mode,
		PME_CLASSIFICATION_MODE classification_mode, uint16_t min_influence,
		uint16_t max_influence) {
	PME->gcr = (global_context & PME_GCR_GLOBAL_MASK) | distance_mode;
	PME->nsr |= classification_mode;
	PME->minif = min_influence;
	PME->maxif = max_influence;
}

void pme_set_global_context(uint8_t global_context) {
	PME->gcr = (PME->gcr & ~PME_GCR_GLOBAL_MASK) | (global_context & PME_GCR_GLOBAL_MASK);
}

void pme_set_distance_mode(PME_DISTANCE_MODE distance_mode) {
	PME->gcr = (PME->gcr & ~PME_GCR_DIST_MASK) | distance_mode;
}

void pme_set_classification_mode(PME_CLASSIFICATION_MODE classification_mode) {
	PME->nsr = (PME->nsr & ~PME_NSR_CLASS_MODE_MASK) | classification_mode;
}

uint8_t pme_get_global_context() {
	return (PME->gcr & PME_GCR_GLOBAL_MASK);
}

PME_DISTANCE_MODE pme_get_distance_mode() {
	return ((PME_DISTANCE_MODE) (PME->gcr & PME_GCR_DIST_MASK));
}

PME_CLASSIFICATION_MODE pme_get_classification_mode() {
	return ((PME_CLASSIFICATION_MODE) (PME->nsr & PME_NSR_CLASS_MODE_MASK));
}

uint8_t pme_get_comitted_neurons() {
	return (PME->forget_ncount);
}

