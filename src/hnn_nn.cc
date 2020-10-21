/*
 * File name: hnn_nn.cc
 * Date:      2020/08/11
 * Author:    Jindriska Deckerova, Jan Faigl
 */

#include <cmath>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>
#include <boost/numeric/ublas/operation.hpp>
#include <utility>
#include <eigen3/Eigen/Dense>
#include <cstdio>

#include <crl/random.h>

#include "hnn_nn.h"

#define CEIL 0.4999999
#define FLOOR 0.0000001
#define DEF 0.01
#define MAX 0.99

using namespace hnn;
using namespace crl;

/// - constructor ---------------------------------------------------------------
CNN::CNN(crl::CConfig &config, CGraph *graph, int size) : graph(graph), n(size)
{
    states.resize(n);
    for (int i = 0; i < n; i++)
    {
        this->states[i].resize(n + 1);
    }
    this->minimaCounter.resize(n);
    for (int i = 0; i < n; i++)
    {
        this->minimaCounter[i].resize(n);
    }

    // init params
    a = config.get<int>("param-a");
    b = config.get<int>("param-b");
    c = config.get<int>("param-c");
    d = config.get<int>("param-d");
    e = config.get<int>("param-e");
    f = config.get<int>("param-f");
    deltaT = config.get<double>("param-t");
    deltaVartheta = config.get<double>("param-theta");

    orig_f = f;
}

/// - destructor ---------------------------------------------------------------
CNN::~CNN()
{
    this->states.clear();
    this->minimaCounter.clear();
}
/// - public method ------------------------------------------------------------
void CNN::init()
{
    float floor = FLOOR;
    float ceil = CEIL;
    float range = ceil - floor;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            states[i][j] = floor + float((range * rand()) / (RAND_MAX + 1.0));
            minimaCounter[i][j] = 0;
        }
        states[i][n] = 0;
    }

    f = orig_f;
}

/// - public method ------------------------------------------------------------
bool CNN::checkLocalMinima()
{
    bool is_minimum = false;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if (minimaCounter[i][j] >= 3)
            {
                is_minimum = true;
                break;
            }
        }
    }
    return is_minimum;
}

/// - public method ------------------------------------------------------------
void CNN::resetLocalMinima()
{
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            minimaCounter[i][j] = 0;
        }
    }
}

/// - public method ------------------------------------------------------------
void CNN::applyFilter()
{
    int largest_i = 0;
    int largest_j = 0;
    double largest_state = 0.0;

    for (int j = 0; j < n; j++)
    {
        for (int i = 0; i < n; i++)
        {
            if (states[i][j] > largest_state)
            {
                largest_state = states[i][j];
                largest_i = i;
                largest_j = j;
            }
            states[i][j] = 0;
        }
        states[largest_i][largest_j] = 1;
        largest_state = 0.0;
    }
}

/// - public method ------------------------------------------------------------
void CNN::adjustNetwork(const TargetPtrVector route, const bool isFeasible)
{
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            states[i][j] = DEF;
        }
    }
    for (int k = 0; k < (int)route.size(); ++k)
    {
        states[route[k]->label][k] = MAX;
        states[route[k]->label][n] = 1.00;
    }
    for (int l = 0; l < n - 1; ++l)
    {
        int rand_idx = crl::CRandom::random() % 10;
        states[l][rand_idx] = MAX;
    }

    if (isFeasible)
    {
        f -= 1;
    }
    else
    {
        f += 1;
    }
}

/// - public method ------------------------------------------------------------
void CNN::updateRandomRow()
{
    crl::CRandom::randomize();
    int row = crl::CRandom::random() % n;
    updateStatesInRow(row);
}

/// - private method ------------------------------------------------------------
void CNN::updateStatesInRow(const int row)
{
    double a_term, b_term, c_term, gamma_term, g_term, f_term, derivation;
    long double new_state;

    double b_sum = bTermSum();
    double k_summation = cTermSumModified();

    for (int j = 0; j < n; j++)
    {
        a_term = a * aTermSum(row, j);
        b_term = b * b_sum;
        c_term = c * gammeFunc(k_summation) * rhoFuncModified(row, j);
        gamma_term = d * lambdaFunc(row, j);
        g_term = e * varepsilonFunc(row, j);
        f_term = f * zetaFunc(row, j);
        derivation = a_term + b_term + c_term - gamma_term + g_term - f_term;
        states[row][j] = updateState(derivation, row, j);
    }

    states[graph->SDEPOT][0] = 1.0;
    states[graph->EDEPOT][n - 1] = 1.0;
}

/// - private method ------------------------------------------------------------
double CNN::aTermSum(const int i, const int j)
{
    double sum = 0.0;
    for (int h = 0; h < n; ++h)
    {
        sum += states[h][j];
    }
    sum -= states[i][j];
    return sum;
}

/// - private method ------------------------------------------------------------
double CNN::bTermSum()
{
    double sum = 0.0;
    for (int h = 0; h < n; h++)
    {
        for (int k = 0; k < n; k++)
        {
            sum += states[h][k];
        }
    }
    return sum - n;
}

/// - private method ------------------------------------------------------------
double CNN::cTermSum()
{
    double k_summation = 0.0;
    int size = n;

    Eigen::MatrixXf matrix_a(size, size - 1);
    Eigen::MatrixXf matrix_b(size - 1, size);
    Eigen::MatrixXf matrix_prod(size, size);

    unsigned int h = 0, q = 0;
    unsigned int size1 = size;
    unsigned int size2 = size - 1;

    for (h = 0; h < size1; h++)
    {
        for (q = 0; q < size2; q++)
        {
            matrix_a(h, q) = (float)states[h][q];
        }
    }

    for (h = 0; h < size2; h++)
    {
        for (q = 0; q < size1; q++)
        {
            matrix_b(h, q) = (float)states[q][h + 1];
        }
    }

    matrix_prod = matrix_a * matrix_b;

    unsigned int from, middle, to;

    for (unsigned int h = 0; h < size; h++)
    {
        for (unsigned int q = 0; q < size; q++)
        {
            from = h > 0 ? (q > 0 ? h - 1 : q) : h;
            middle = h;
            to = q;
            const double dist = graph->getDistance(from, middle, to);
            k_summation += matrix_prod.coeff(h, q) * dist;
        }
    }

    k_summation -= graph->budget;

    return k_summation;
}

/// - private method ------------------------------------------------------------
double CNN::cTermSumModified()
{
    double sum = 0.0;
    Eigen::MatrixXf matrix_k(n, n - 1);
    Eigen::MatrixXf matrix_i(n - 1, n - 1);
    Eigen::MatrixXf matrix_h(n - 1, n);
    Eigen::MatrixXf matrix_prod(n, n);

    for (int j = 0; j < n - 1; j++)
    {
        for (int i = 0; i < n - 1; i++)
        {
            matrix_i(j, i) = states[i][j];
        }
    }
    for (int j = 1; j < n; j++)
    {
        for (int k = 0; k < n; k++)
        {
            matrix_k(k, j - 1) = states[k][j - 1];
        }
    }

    for (int j = 0; j < n - 1; j++)
    {
        for (int h = 0; h < n; h++)
        {
            matrix_h(j, h) = states[h][j + 1];
        }
    }
    Eigen::MatrixXf tmp = matrix_k * matrix_i;
    matrix_prod = tmp * matrix_h;

    for (unsigned int i = 0; i < n; i++)
    {
        for (unsigned int k = 0; k < n; k++)
        {
            for (unsigned int h = 0; h < n; h++)
            {
                sum += graph->getDistance(k, i, h) * matrix_prod(k, h);
            }
        }
    }
    return sum - graph->budget;
}

/// - private method ------------------------------------------------------------
double CNN::gammeFunc(const double sum)
{
    return sum >= 0 ? sum : 0.0;
}

/// - private method ------------------------------------------------------------
double CNN::rhoFunc(const int i, const int j)
{
    int size = n;
    double q_summation = 0.0;
    unsigned int from, middle, to;

    if (j > 0 && j < size - 1)
    {
        for (int k = 0; k < size; k++)
        {
            for (int h = 0; h < size; ++h)
            {
                from = i > 0 ? (h > 0 ? i - 1 : h) : i;
                middle = i;
                to = h;
                q_summation += graph->getDistance(from, middle, to) * (states[h][j - 1] + states[h][j + 1]);
            }
        }
    }
    else if (j == size - 1)
    {
        for (int h = 0; h < size; ++h)
        {
            from = i > 0 ? (h > 0 ? i - 1 : h) : i;
            middle = i;
            to = h;
            q_summation += graph->getDistance(i, h) * states[h][j - 1];
        }
    }
    else if (j == 0)
    {
        for (int h = 0; h < size; ++h)
        {
            from = i > 0 ? (h > 0 ? i - 1 : h) : i;
            middle = i;
            to = h;
            q_summation += graph->getDistance(i, h) * states[h][j + 1];
        }
    }

    return q_summation;
}

/// - private method ------------------------------------------------------------
double CNN::rhoFuncModified(const int i, const int j)
{
    int size = n;
    double q_summation = 0.0;
    unsigned int from, middle, to;

    double sum1 = 0.0;
    for (int p = 0; p < size; p++)
    {
        for (int h = 0; h < size; h++)
        {
            sum1 += graph->getDistance(i, p, h) * states[p][j - 1] * states[h][j];
        }
    }

    double sum2 = 0.0;
    for (int k = 0; k < size; k++)
    {
        for (int h = 0; h < size; h++)
        {
            sum2 += graph->getDistance(k, i, h) * states[k][j - 1] * states[h][j];
        }
    }

    double sum3 = 0.0;
    for (int p = 0; p < size; p++)
    {
        for (int k = 0; k < size; k++)
        {
            sum3 += graph->getDistance(k, p, i) * states[k][j] * states[p][j + 1];
        }
    }

    return sum1 + sum2 + sum3;
}

/// - private method ------------------------------------------------------------
double CNN::lambdaFunc(const int i, const int j)
{
    return ((i == 0 && j == 0) || (i == n - 1 && j == n - 1)) ? 1.0 : 0.0;
}

/// - private method ------------------------------------------------------------
double CNN::varepsilonFunc(const int i, const int j)
{
    if (j == n)
    {
        double summation = 0.0;
        for (int h = 0; h < n; ++h)
        {
            summation += states[i][h];
        }
        return 1 - summation;
    }
    return (-1) * states[i][n];
}

/// - private method ------------------------------------------------------------
double CNN::zetaFunc(const int i, const int j)
{
    return j == n ? graph->getReward(i) : 0;
}

/// - private method ------------------------------------------------------------
long double CNN::updateState(const double value, const int i, const int j)
{
    double state, delta_i, curr_input, new_input, tmp_e, exponent;
    long double new_output;

    state = states[i][j];
    delta_i = (-1) * value * deltaT;
    curr_input = state == 0 ? 0 : log(state) - log(1 - state);
    new_input = curr_input + delta_i;
    tmp_e = new_input * (-1);
    exponent = exp(tmp_e);
    new_output = 1 / (1 + exponent);

    if (abs(delta_i) < deltaVartheta)
    {
        minimaCounter[i][j] += 1;
    }
    else
    {
        minimaCounter[i][j] = 0;
    }

    return new_output;
}