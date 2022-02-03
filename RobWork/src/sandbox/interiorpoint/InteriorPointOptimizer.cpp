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

/*
 * InteriorPointOptimizer.cpp
 *
 *  Created on: Jan 15, 2009
 *      Author: lpe
 */

#include "InteriorPointOptimizer.hpp"

using namespace rw::math;

namespace internal {
    void compute_f_info_EXT(const Eigen::VectorXd& x, double &f, Eigen::VectorXd &df, Eigen::MatrixXd &ddf) {

    }

    void compute_con_info_i_EXT(int i,
                                const Eigen::VectorXd& x,
                                Eigen::VectorXd &a,
                                Eigen::MatrixXd &da,
                                Eigen::MatrixXd &dda) {

    }

}

void InteriorPointOptimizer::initialize() {
    _x = Eigen::VectorXd(N);
    _s = Eigen::VectorXd(M);
    _z = Eigen::VectorXd(M);
    _df = Eigen::VectorXd(N);
    _ddf = Eigen::MatrixXd(N,N);
    _a = Eigen::VectorXd(M);
    _da = Eigen::MatrixXd(M,N);
    _dda = Eigen::MatrixXd(N,N);
    _accuracy = 1e-9;
}

InteriorPointOptimizer::InteriorPointOptimizer(size_t n,
                                               size_t m,
                                               ObjectFunction objectFunction,
                                               ConstraintFunction constraintFunction):
    compute_f_info_EXT(objectFunction),
    compute_con_info_i_EXT(constraintFunction),
    N(n),
    M(m)
{
   // compute_f_info_EXT = boost::bind(&InteriorPointOptimizer::objectFunction, this, _1, _2, _3, _4);
   // compute_con_info_i_EXT = boost::bind(&InteriorPointOptimizer::constraintFunction, this, _1, _2, _3, _4, _5);
    initialize();
}


InteriorPointOptimizer::InteriorPointOptimizer(size_t n, size_t m):
    compute_f_info_EXT(boost::bind(&InteriorPointOptimizer::objectFunction, this, _1, _2, _3, _4)),
    compute_con_info_i_EXT(boost::bind(&InteriorPointOptimizer::constraintFunction, this, _1, _2, _3, _4, _5)),
    N(n),
    M(m)
{
    initialize();
}


InteriorPointOptimizer::~InteriorPointOptimizer()
{

}

void InteriorPointOptimizer::objectFunction(const Eigen::VectorXd& x,
                                            double &f,
                                            Eigen::VectorXd &df,
                                            Eigen::MatrixXd &ddf) {
    RW_THROW("Object function undefined for InteriorPointMethod");
}

void InteriorPointOptimizer::constraintFunction(const Eigen::VectorXd& x,
                                                int i,
                                                Eigen::VectorXd &a,
                                                Eigen::MatrixXd &da,
                                                Eigen::MatrixXd &dda)
{
    RW_THROW("Constraint function undefined for InteriorPointMethod");
}



void InteriorPointOptimizer::setAccuracy(double accuracy) {
    _accuracy = accuracy;
}

double InteriorPointOptimizer::getAccuracy() {
    return _accuracy;
}



// Verification of objective and constraints (needs an admissible x)
void InteriorPointOptimizer::verify_user_defined_objective_and_constraints() {
    double h,delta,ftest,fold,atest;
    Eigen::VectorXd xtest(N),dxtest(N);
    Eigen::MatrixXd ddfold(N,N);
    Eigen::VectorXd dfold(N);
    Eigen::MatrixXd ddaold(N,N);
    Eigen::MatrixXd daold(M,N);
    Eigen::VectorXd aold(M);

    for(size_t j=0; j<M; j++) {
        compute_con_info_i_EXT(_x, j, _a, _da, _dda);
        if (_a(j)<=0)
            RW_THROW("Initial value: Violation of constraint ");
        else
            std::cout << "Constraint " << j << " has value " << _a(j)<<std::endl;
    }
    delta=0.0;
    h=0.00001;
    for(size_t i=0; i<N; i++) {
        xtest(i) = delta*rand()/32768.0;
        dxtest(i) = h*rand()/32768.0;
    }
    _x += xtest;
    compute_f_info_EXT(_x, fold, dfold, ddfold);
    _x += dxtest;
    compute_f_info_EXT(_x, _f, _df, _ddf);
    ftest = fold+inner_prod(dfold,dxtest);
    std::cout << "Objective. " << "First order:" << "h: "<< h<< " Rel. error:"<<(ftest-_f)/(_f-fold)<<std::endl;
    for (size_t jj = 0; jj < N; ++jj)
        for (size_t kk = 0; kk < N; ++kk)
            ftest+=0.5*_ddf(jj,kk)*dxtest(jj)*dxtest(kk);
    std::cout<< "Objective. " << "Second order:" << "h: "<< h<< " Rel. error:"<<(ftest-_f)/(_f-fold)<<std::endl;
    _x -= dxtest;

    for(size_t j=0; j<M; j++) {
        compute_con_info_i_EXT(_x, j, aold, daold, ddaold);
        _x += dxtest;
        compute_con_info_i_EXT(_x, j, _a, _da, _dda);
        atest=aold(j);
        for(size_t i=0; i<N; i++)
            atest += daold(j,i)*dxtest(i);
        std::cout << "Constraint no. " << j <<". First order:" << "h: "<< h<< " Rel. error:"<<(atest-_a(j))/(_a(j)-aold(j))<<std::endl;
        for (size_t jj = 0; jj < N; ++jj)
            for (size_t kk = 0; kk < N; ++kk)
                atest += 0.5*_dda(jj,kk)*dxtest(jj)*dxtest(kk);
        std::cout << "Constraint no. " << j <<". Second order:" << "h: "<< h<< " Rel. error:"<<(atest-_a(j))/(_a(j)-aold(j))<<std::endl;
        _x -= dxtest;
    }
}



void InteriorPointOptimizer::choleskySolve(int n_e, int bw, Eigen::MatrixXd &A, Eigen::VectorXd &b, Eigen::VectorXd &x) {

    // "n_e" is the dimension of the system. "bw" is the bandwidth measured as the max-distance
    // from the diagonal of a non-zero element in A.
    Eigen::VectorXd y(n_e);
    int ist,jst;

    for (int k = 0; k < n_e; ++k) {
        y(k) = b(k);
        jst=k-bw; if (jst<0) jst=0;
        for (int jj = jst; jj < k; ++jj) {
            A(k,k)-=A(k,jj)*A(k,jj);
            y(k)-=A(k,jj)*y(jj);
        }
        A(k,k)=sqrt(A(k,k));
        y(k)/=A(k,k);
        ist=k+bw+1; if (ist>n_e) ist=n_e;
        for (int ii = k+1; ii < ist; ++ii) {
            jst=k-bw; if (jst<0) jst=0;
            for (int jj = jst; jj < k; ++jj)
                A(ii,k)-=A(ii,jj)*A(k,jj);
            A(ii,k)=A(ii,k)/A(k,k);
        }
    }

    for (int k = n_e-1; k >=0; --k) {
            x(k)=y(k);
            ist=k+bw+1; if (ist>n_e) ist=n_e;
            for (int ii = k+1; ii < ist; ++ii)
                x(k)-=A(ii,k)*x(ii);
            x(k)/=A(k,k);
    }
}


// Computation of objective contributions to the linear equations
void InteriorPointOptimizer::compute_f_info(Eigen::MatrixXd &A, Eigen::VectorXd &RHS) {

    compute_f_info_EXT(_x,  _f, _df, _ddf);
    for (size_t jj = 0; jj < N; ++jj) {
        for (size_t kk = 0; kk < N; ++kk)
            A(jj, kk) += _ddf(jj, kk);
        RHS(jj) -= _df(jj);
    }

}


// Computation of constraint contributions to the linear equations
void InteriorPointOptimizer::compute_con_info(Eigen::MatrixXd &A, Eigen::VectorXd &RHS) {

    double zs;

    for (size_t i = 0; i < M; ++i) {
        compute_con_info_i_EXT(_x, i, _a, _da, _dda);
        zs = _z(i)/_s(i);
        for (size_t jj = 0; jj < N; ++jj) {
            for (size_t kk = 0; kk < N; ++kk)
                A(jj,kk) += -_z(i)*_dda(jj,kk)+zs*_da(i,jj)*_da(i,kk);
            RHS(jj) += (_mu/_s(i)-zs*_a(i)+_z(i))*_da(i,jj);
        }
    }

}

// merit_info and differentiated merit_info implements the merit function given by Eq. 19.26 in Nocedal and Wright
void InteriorPointOptimizer::merit_info(Eigen::VectorXd &x, Eigen::VectorXd &s, double &phi, double &eta){
    phi = 0;
    phi +=_f;
    for (size_t i = 0; i < M; ++i) {
        if (_a(i)>s(i))
            phi+=eta*(_a(i)-s(i))-_mu*log(s(i));
        else
            phi+=eta*(s(i)-_a(i))-_mu*log(s(i));
    }
}

void InteriorPointOptimizer::Dmerit_info(Eigen::VectorXd &x, Eigen::VectorXd &s, Eigen::VectorXd &dx, Eigen::VectorXd &ds, double &Dphi, double &eta) {

    Dphi=0;
    for (size_t i = 0; i < N; ++i)
        Dphi += _df(i)*dx(i);

    for (size_t j = 0; j < M; ++j) {
        Dphi-= _mu*ds(j)/s(j);
        if (_a(j)>s(j)) {
            Dphi-=eta*ds(j);
            for (size_t i = 0; i < N; ++i)
                Dphi += eta*_da(j,i)*dx(i);
        }
        else {
            Dphi+=eta*ds(j);
            for (size_t i = 0; i < N; ++i)
                Dphi -= eta*_da(j,i)*dx(i);
        }

    }
}



// trust region based update. Method used is Nocedal and Wright p. 572-577. NOTICS: Adaptive updates of eta for efficiency is still not implemented
void InteriorPointOptimizer::update(Eigen::VectorXd &x, Eigen::VectorXd &dx, Eigen::VectorXd &s, Eigen::VectorXd &z) {
    double phi0,phi,Dphi;
    Eigen::VectorXd x0(N);
    Eigen::VectorXd s0(M),ds(M);
    double tmax,t,oldt,fval,oldfval,eps;
    size_t i,jj;


    _eta=1;
    eps=0.005;

    // Controlling not to pass constraint boundaries
    tmax=1;
    for (i = 0; i < M; ++i) {
        ds(i) = _a(i)-s(i);
        for (jj = 0; jj < N; ++jj)
            ds(i) += _da(i,jj)*dx(jj);
        if((-ds(i)*tmax)>s(i)) tmax=-s(i)/ds(i);
    }
    tmax*=(1-eps);

    // Merit function is Formula 19.26 in [1]. We use the merit function for a trust region.
    merit_info(x,s,phi0,_eta);
    Dmerit_info(x,s,dx,ds,Dphi,_eta);

    x0=x;
    s0=s;
    t=tmax;
    x=x0+t*dx;
    s=s0+t*ds;
    compute_f_info_EXT(x, _f, _df, _ddf);
    for (size_t j = 0; j < M; ++j)
        compute_con_info_i_EXT(x, j, _a, _da, _dda);
    merit_info(x,s,phi,_eta);
    fval=(phi-phi0-0.8*t*Dphi)/(phi0-phi);
    if (fval>0) {
        while (fval>0) {
            oldt=t;
            oldfval=fval;
            t*=0.8;
            x=x0+t*dx;
            s=s0+t*ds;
            compute_f_info_EXT(x, _f, _df, _ddf);
            for (size_t j = 0; j < M; ++j)
                compute_con_info_i_EXT(x, j, _a, _da, _dda);
            merit_info(x,s,phi,_eta);
            fval=(phi-phi0-0.8*t*Dphi)/(phi0-phi);
        }
    }
    if (t>tmax) {
        std::cout << "t>tmax" << t<<tmax<<std::endl; exit(1);
    }
    for (size_t j = 0; j < M; ++j)
        z(j)=_mu/s(j);


}


int InteriorPointOptimizer::solve(const Eigen::VectorXd& x_init) {
    _x = x_init;
    Eigen::MatrixXd A(N, N);
    Eigen::VectorXd RHS(N);
    Eigen::VectorXd dx(N);
    Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(N, N);
    Eigen::VectorXd RHS0 = Eigen::Vector2d::Zero(N);
    Eigen::VectorXd oldx(N),diffx(N);
    int n_eq_solves=0;
    int bw=N;
    double testval;
    _mu=0.01;

    // user defined initialization of primal variables
    //initialize_EXT();

    // initialization of dual variables
    for (size_t k = 0; k < M; ++k) {
        compute_con_info_i_EXT(_x, k, _a, _da, _dda);
        _s(k) = _a(k);
        _z(k) = 0;
    }

    // verification of user defined routines
    verify_user_defined_objective_and_constraints();


    // iterations for solving the optimization problem
    double diff=1;
    while(diff>(_accuracy*_accuracy)&& _mu>_accuracy) {
        oldx = _x;
        testval=1;
        while(testval>_mu*_mu) {
            A = A0;
            RHS = RHS0;
            compute_f_info(A, RHS);

            compute_con_info(A, RHS);

            choleskySolve(N,bw,A,RHS,dx);
            n_eq_solves++;

            update(_x,dx,_s,_z);

            testval=inner_prod(dx,dx);
        }
        diffx = _x-oldx;
        diff=inner_prod(diffx,diffx);
        // update of mu. NOTICE: Should be made adaptive for efficiency
        _mu*=0.1;
    }
    std::cout <<"Search finished. Result is "<< _x << "The accuracy is at least "<< sqrt(diff) <<". The number of LE solves was "<<n_eq_solves<<std::endl;

    return 0;
}
