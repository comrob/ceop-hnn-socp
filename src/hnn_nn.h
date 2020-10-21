/*
 * File name: hnn_nn.h
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#ifndef __HNN_NN_H__
#define __HNN_NN_H__

#include <crl/config.h>

#include <vector>

#include "hnn_graph.h"
#include "target.h"

typedef std::vector<std::vector<double>> Double2Vector;
typedef std::vector<std::vector<int>> Int2Vector;

namespace hnn
{
    class CNN
    {
    public:
        static crl::CConfig &getConfig(crl::CConfig &config);
        CNN(crl::CConfig &config, CGraph *graph, int size);
        ~CNN();
        Double2Vector getStates() const { return states; }
        void init();
        bool checkLocalMinima();
        void resetLocalMinima();
        void applyFilter();
        void adjustNetwork(const TargetPtrVector route, const bool isFeasible);
        void updateRandomRow();

    private:
        void updateStatesInRow(const int row);
        double aTermSum(const int i, const int j);
        double bTermSum();
        double cTermSum();
        double cTermSumModified();
        double gammeFunc(const double sum);
        double rhoFunc(const int i, const int j);
        double rhoFuncModified(const int i, const int j);
        double lambdaFunc(const int i, const int j);
        double varepsilonFunc(const int i, const int j);
        double zetaFunc(const int i, const int j);
        long double updateState(const double value, const int i, const int j);

    private:
        Double2Vector states;
        Int2Vector minimaCounter;
        CGraph *graph;
        const int n; // size

        // params
        int a, b, c, d, e, f, orig_f;
        double deltaT, deltaVartheta;
    };
} // end namespace hnn

#endif //__HNN_NN_H__