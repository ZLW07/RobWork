/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWLIBS_ALGORITHMS_RANSACMODEL_HPP
#define RWLIBS_ALGORITHMS_RANSACMODEL_HPP

/**
 * @file RANSACModel.hpp
 */

#include <rw/core/Ptr.hpp>
#include <rw/math/Random.hpp>

#include <algorithm>
#include <vector>

namespace rwlibs { namespace algorithms {

    /**
     * @brief An interface for RANSAC model fitting.
     *
     * @todo A model needs to remember the indices of inliers from the set of data...
     */
    template< class MODEL, class DATA > class RANSACModel
    {
      public:
        //! @brief Smart pointer type to this class.
        typedef rw::core::Ptr< RANSACModel< MODEL, DATA > > Ptr;

      public:
        //! @brief Constructor.
        RANSACModel () : _quality (0.0) {}

        /**
         * @brief Creates a new model of this type using provided data.
         */
        virtual MODEL make (const std::vector< DATA >& data) const
        {
            MODEL newModel;
            newModel.refit (data);
            return newModel;
        }

        //! @brief Destructor.
        virtual ~RANSACModel () {}

        /**
         * @brief Find models fitting a set of observations.
         *
         * Function performs \b maxIterations iterations and finds a number of initial models. The
         * models are then filtered, and those within a distance of \b modelThreshold are merged
         * together.
         *
         * @param data [in] set of data points to find a model for
         * @param maxIterations [in] number of iterations to perform
         * @param dataRequired [in] a number of data points required to fit into model to consider
         * it plausible
         * @param dataThreshold [in] a threshold for fitting error of a data point
         * @param modelThreshold [in] a difference between models neccesary to consider them
         * different
         *
         * @return a vector of fitted models
         */
        static std::vector< MODEL > findModels (const std::vector< DATA >& data, int maxIterations,
                                                int dataRequired, double dataThreshold,
                                                double modelThreshold)
        {
            int n = MODEL ().getMinReqData ();

            if (data.size () < static_cast< size_t > (n) ||
                data.size () < static_cast< size_t > (dataRequired)) {
                // RW_WARN("Too few samples to create a proper model.");

                return std::vector< MODEL > ();
            }

            // create a vector of indices used for shuffling and picking random points so they don't
            // repeat
            std::vector< size_t > indices;
            for (size_t i = 0; i < data.size (); ++i) {
                indices.push_back (i);
            }
            std::vector< std::pair< MODEL, int > >
                models;    // pair containing a model and a number of inliers

            int iterations = 0;
            while (iterations++ < maxIterations) {
                std::random_shuffle (indices.begin (), indices.end ());
                std::vector< DATA > maybeInliers;
                for (size_t i = 0; i < static_cast< size_t > (n); ++i) {
                    maybeInliers.push_back (data[indices[i]]);
                }

                // create a model based on the maybeInliers
                MODEL maybeModel;
                try {
                    maybeModel = maybeModel.make (maybeInliers);

                    if (maybeModel.invalid ()) {
                        continue;
                    }
                }
                catch (...) {
                    continue;
                }

                // add the maybe inliers to the conensus set
                std::vector< DATA > consensusSet (maybeInliers);

                // check if any point in data fits the model with an error smaller than t
                for (size_t i = 0; i < data.size (); i++) {
                    if (maybeModel.fitError (data[i]) < dataThreshold) {
                        consensusSet.push_back (data[i]);
                    }
                }

                // if consensus set size is large enough, we have a model
                if (consensusSet.size () > static_cast< size_t > (dataRequired)) {
                    models.push_back (std::pair< MODEL, int > (
                        maybeModel, static_cast< int > (consensusSet.size ())));
                }
            }

            // merging models
            if (models.size () == 0) {
                // if no models found, return empty vector
                return std::vector< MODEL > ();
            }

            if (models.size () == 1) {
                // re-fit data to the single
                std::vector< DATA > consensusSet;
                std::vector< size_t > consensusSetIndices;
                for (size_t k = 0; k < data.size (); ++k) {
                    if (models[0].first.belongsTo (data[k], dataThreshold)) {
                        consensusSet.push_back (data[k]);
                        consensusSetIndices.push_back (k);
                    }
                }

                try {
                    models[0].first.refit (consensusSet);
                    models[0].first._indices = consensusSetIndices;

                    if (models[0].first.invalid () || models[0].first.getNumberOfInliers () <
                                                          static_cast< size_t > (dataRequired)) {
                        return std::vector< MODEL > ();
                    }
                }
                catch (...) {
                    return std::vector< MODEL > ();
                }

                return std::vector< MODEL > (1, models[0].first);
            }

            // merge models that are closely related
            std::vector< std::pair< MODEL*, int > > modelsPtr (models.size ());

            for (size_t i = 0; i < models.size (); i++) {
                modelsPtr[i].first  = &(models[i].first);
                modelsPtr[i].second = models[i].second;
            }

            bool merging = false;
            do {
                merging = false;

                for (size_t i = 0; i < modelsPtr.size () - 1; i++) {
                    if (modelsPtr[i].first == NULL) {
                        continue;
                    }

                    // compare with all following models...
                    std::pair< MODEL*, int > bestCloseModel (modelsPtr[i].first,
                                                             modelsPtr[i].second);
                    for (size_t j = i + 1; j < modelsPtr.size (); j++) {
                        if (modelsPtr[j].first == NULL) {
                            // the model was looked at already
                            continue;
                        }

                        // (disregard, if those models are different)
                        bool res = models[i].first.same (models[j].first, modelThreshold);
                        if (!res) {
                            continue;
                        }

                        // ...to see which model has more inliers
                        // merge those models
                        if (bestCloseModel.second < modelsPtr[j].second) {
                            bestCloseModel = modelsPtr[j];
                        }

                        // mark the model as processed
                        modelsPtr[j].first = NULL;

                        merging = true;
                    }

                    if (bestCloseModel.first == NULL) {
                        continue;
                    }

                    // re-fit data to the best close model found
                    std::vector< DATA > consensusSet;
                    std::vector< size_t > consensusSetIndices;
                    for (size_t k = 0; k < data.size (); ++k) {
                        if (bestCloseModel.first->belongsTo (data[k], dataThreshold)) {
                            consensusSet.push_back (data[k]);
                            consensusSetIndices.push_back (k);
                        }
                    }

                    try {
                        bestCloseModel.first->refit (consensusSet);
                        bestCloseModel.first->_indices = consensusSetIndices;

                        if (bestCloseModel.first->invalid () ||
                            bestCloseModel.first->getNumberOfInliers () <
                                static_cast< size_t > (dataRequired)) {
                            continue;
                        }
                    }
                    catch (...) {
                        continue;
                    }
                }
            } while (merging);

            std::vector< MODEL > newModels;
            for (size_t i = 0; i < modelsPtr.size () - 1; i++) {
                if (modelsPtr[i].first != NULL) {
                    newModels.push_back (*modelsPtr[i].first);
                }
            }

            return newModels;
        }

        /**
         * @brief Select the model with the largest number of inliers.
         *
         * In case of ties, pick the model with better quality.
         */
        static MODEL bestModel (const std::vector< MODEL >& models)
        {
            if (models.size () == 0) {
                return MODEL ();
            }

            size_t idx = 0;

            for (size_t i = 0; i < models.size (); ++i) {
                if (models[idx] < models[i]) {
                    idx = i;
                }
            }

            return models[idx];
        }

        /**
         * @brief Select a model randomly, with a chance based on the number of inliers.
         *
         * Given a vector of models {model1(45 inliers), model2(30 inliers), model3(20 inliers),
         * model4(5 inliers)}, it will pick:
         * - model1 - 45% chance,
         * - model2 - 30% chance,
         * - model3 - 20% chance,
         * - model4 - 5% chance
         */
        static MODEL likelyModel (const std::vector< MODEL >& models)
        {
            if (models.size () == 0) {
                return MODEL ();
            }

            size_t total_inliers = 0;
            std::vector< size_t > thresholds;

            for (size_t i = 0; i < models.size (); ++i) {
                size_t inliers = models[i].getNumberOfInliers ();

                total_inliers += inliers;
                thresholds.push_back (total_inliers);
            }

            size_t pick = rw::math::Random::ran (0.0, 1.0) * total_inliers;

            size_t idx = 0;    // model index
            for (size_t i = 0; i < models.size (); ++i) {
                if (pick < thresholds[i]) {
                    idx = i;
                    break;
                }
            }

            return models[idx];
        }

        /**
         * @brief 'Worse than' operator.
         *
         * Used for sorting. Compares the number of inliers the models have.
         * In case of ties, compares quality.
         */
        bool operator< (const MODEL& model) const
        {
            size_t a = getNumberOfInliers ();
            size_t b = model.getNumberOfInliers ();

            if (a < b) {
                return true;
            }
            else if (a == b) {
                return getQuality () > model.getQuality ();
            }
            else {
                return false;
            }
        }

        /**
         * @brief 'Better than' operator.
         *
         * Used for sorting. Compares the number of inliers the models have.
         * In case of ties, compares quality.
         */
        bool operator> (const MODEL& model) const
        {
            size_t a = getNumberOfInliers ();
            size_t b = model.getNumberOfInliers ();

            if (a > b) {
                return true;
            }
            else if (a == b) {
                return getQuality () < model.getQuality ();
            }
            else {
                return false;
            }
        }

        /**
         * @brief Calculates the fitting error of a sample.
         */
        virtual double fitError (const DATA& sample) const = 0;

        /**
         * @brief Check whether a sample belongs to the model.
         *
         * Returns \b true when the sample is within a threshold distance of the model.
         */
        virtual bool belongsTo (const DATA& sample, double threshold) const
        {
            return fitError (sample) <= threshold;
        }

        /**
         * @brief Checks whether the model is invalid.
         */
        virtual bool invalid () const = 0;

        /**
         * @brief Recalculates the model based on provided samples.
         *
         * @return Fit error on a set of provided samples.
         */
        virtual double refit (const std::vector< DATA >& data) = 0;

        /**
         * @brief Returns the number of samples required to create the model.
         *
         * No model will be found, if data size is less than this amount.
         */
        virtual int getMinReqData () const = 0;

        /**
         * @brief Tests whether the model is same to a threshold of another model.
         *
         * This is used to filter down (merge) similar models found in the course of RANSAC
         * algorithm execution.
         */
        virtual bool same (const MODEL& model, double threshold) const = 0;

        /**
         * @brief Get the number of inliers.
         */
        size_t getNumberOfInliers () const { return _data.size (); }

        /** @brief Get the model quality.
         *
         * The model quality is a sum of fitting errors for all its inliers.
         */
        double getQuality () const { return _quality; }

        /**
         * @brief Set the model quality.
         *
         * @param quality [in] fitting error of the model
         */
        void setQuality (double quality) { _quality = quality; }

        //! @brief Access data.
        std::vector< DATA >& getData () { return _data; }

        //! @brief Access data.
        const std::vector< DATA >& getData () const { return _data; }

        /**
         * @brief Get the vector of inlier indices
         *
         * @return a vector of indices of inliers in the data vector supplied to findModels
         * function.
         */
        std::vector< size_t > getInlierIndices () const { return _indices; }

      protected:
        // body
        std::vector< DATA > _data;
        double _quality;

        /*
         * For some calculations, it is neccesary to know which data points given to findModels()
         * function were actualy fitted in the model. This vector holds the indices of inliers.
         * Use at your own risk, if you know what you are doing.
         */
        std::vector< size_t > _indices;
    };

}}    // namespace rwlibs::algorithms

#endif    // include guard
