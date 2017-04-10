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
 
#ifndef _NEURONS_H_
#define _NEURONS_H_

#include "stdint.h"

#define PME_NO_MATCH 0x7fff
#define PME_MIN_CONTEXT 0
#define PME_MAX_CONTEXT 127
#define PME_FIRST_NEURON_ID 1
#define PME_LAST_NEURON_ID 128
#define PME_MAX_NEURONS 128
#define PME_MAX_VECTOR_SIZE 128
#ifndef PME_USER_MAX_VECTOR_SIZE
	#define PME_USER_MAX_VECTOR_SIZE PME_MAX_VECTOR_SIZE // e.g. Compile with option -DPME_USER_MAX_VECTOR_SIZE 10 to set the maximum size to the vector to be read/written from and to the PME to 10.  This value should not be set to more that 128.
#endif

#define PME_NCR_ID_MASK 0xFF00	// Upper 8-bit of Neuron ID
#define PME_NCR_NORM_MASK 0x0040		// 1 = LSUP, 0 = L1
#define PME_NCR_CONTEXT_MASK 0x007F	// Neuron Context
#define PME_NCR_CAT_DEGEN_MASK 0x8000	// Indicates neuron is degenerate
#define PME_CAT_CATEGORY_MASK 0x7FFF	// the category associated with a neuron
#define PME_GCR_DIST_MASK 0x0080	// distance type, 1 = Lsup, 0 = L1
#define PME_GCR_GLOBAL_MASK 0x007F	// the context of the neuron, used to segment the network
#define PME_NSR_CLASS_MODE_MASK 0x0020	// Classifier mode 1 = KNN, 0 = RBF (KNN not for learning mode)
#define PME_NSR_SR_MODE_MASK 0x0010	// 1 = SR (save/restore) 0 = LR (learn/recognize)
#define PME_NSR_ID_FLAG_MASK 0x0008	// Indicates positive identification
#define PME_NSR_UNCERTAIN_FLAG_MASK 0x0004	// Indicates uncertain identification

typedef enum {
	RBF_Mode = 0, KNN_Mode = PME_NSR_CLASS_MODE_MASK
} PME_CLASSIFICATION_MODE;

typedef enum {
	L1_Distance = 0, LSUP_Distance = PME_GCR_DIST_MASK
} PME_DISTANCE_MODE;

typedef struct __neuron_data_Str {
	uint8_t context;
	uint16_t influence;
	uint16_t min_influence;
	uint16_t category;
	uint8_t vector[PME_USER_MAX_VECTOR_SIZE];
} neuron_data_Str;

/** Pattern Matching Engine register map. */
typedef struct {
	volatile uint32_t ncr; /** Neuron Context Register */
	volatile uint32_t comp; /** Component Register */
	volatile uint32_t lcomp; /** Last Component */
	volatile uint32_t idx_dist;/** Write Component Index / Read Distance */
	volatile uint32_t cat; /** Category Register */
	volatile uint32_t aif; /** Active Influence Field */
	volatile uint32_t minif; /** Minimum Influence Field */
	volatile uint32_t maxif; /** Maximum Influence Field */
	volatile uint32_t testcomp; /** Write Test Component */
	volatile uint32_t testcat; /** Write Test Category */
	volatile uint32_t nid; /** Network ID */
	volatile uint32_t gcr; /** Global Context Register */
	volatile uint32_t rstchain; /** Reset Chain */
	volatile uint32_t nsr; /** Network Status Register */
	volatile uint32_t reserved; /** reserved */
	volatile uint32_t forget_ncount; /** Forget Command / Neuron Count */
} pme_reg_t;

#define PME_BASE (0xB0600000L)
#define PME ((pme_reg_t *) PME_BASE)

/**
 *  Initialise PME
 */
int pme_begin();

/**
 * Forget PME data
 */
void pme_forget();

/**
 * Learn training vector
 * @param[in]	:	vector to learn
 * @param[in]	:	length of the vector to learn
 * @param[in]	:	category of the training vector
 * @return		: 	number of neurons currently committed
 */
int pme_learn(uint8_t vector[], uint8_t length, uint16_t category);

/**
 * Read the data of the neurons committed in the PME
 * The number of neurons to be read and stored is determined by reading
 * FORGET_NCOUNT register.  This function is to be called when the PME is
 * in Normal mode (i.e. learn or classify mode).
 * @param[out]	:	vector to store the value of the neurons
 * @return		:	number of neurons read
 */
int pme_read_neurons(neuron_data_Str ptr_neurons_data[]);

/**
 * Write the neurons to the PME by using save and restore mode
 */
int pme_write_neurons(const neuron_data_Str ptr_neurons_data[],
		uint8_t number_of_neurons);

/**
 * Classify the vector
 * @param[in]	:	vector to classify
 * @param[in]	:	length of the vector to classify
 * @param[out]	:	distance calculated by the PME for firing neurons, this array should be able to contain uint16_t for (max_firing_neurons) values
 * @param[out]	:	category associated to the distance and neuron_ID, this array should be able to contain uint16_t for (max_firing_neurons) values
 * @param[out]	:	neuron ID associated to the distance and category calculated by the PME, this array should be able to contain uint16_t for (max_firing_neurons) values
 * @param[in]	:	data from the top (max_firing_neurons) neurons will be returned in distance, category and neuron_ID. If (max_firing_neurons) is less than the actual number of firing neurons the number of firing neurons is returned
 * @return		:	number of firing neurons whose data are returned
 */
int pme_classify(const uint8_t vector[], uint8_t length, uint16_t distance[],
		uint16_t category[], uint8_t neuron_id[], uint8_t max_firing_neurons);

/**
 * Configure the PME.
 * @param[in]	:	global context of the PME
 * @param[in]	:	distance mode to be used by the PME (either L1 or Lsup)
 * @param[in]	:	classification mode to be used by the PME (either RBF or KNN)
 * @param[in]	:	minimum influence field of neurons
 * @param[in]	: 	maximum influence field of neurons
 */
void pme_configure(uint8_t global_context, PME_DISTANCE_MODE distance_mode,
		PME_CLASSIFICATION_MODE classification_mode, uint16_t min_influence,
		uint16_t max_influence);

void pme_set_global_context(uint8_t global_context);

void pme_set_distance_mode(PME_DISTANCE_MODE distance_mode);

void pme_set_classification_mode(PME_CLASSIFICATION_MODE classification_mode);

uint8_t pme_get_global_context();

PME_DISTANCE_MODE pme_get_distance_mode();

PME_CLASSIFICATION_MODE pme_get_classification_mode();

uint8_t pme_get_comitted_neurons();

#endif
