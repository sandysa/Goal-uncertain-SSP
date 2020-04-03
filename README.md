# Goal uncertain Stochastic shortest path problem

C++ codebase for "Planning in Stochastic Environments with Goal Uncertainty", published at IROS 2019.
Authors: Sandhya Saisubramanian, Kyle Hollins Wray, Luis Pineda, and Shlomo Zilberstein

Link to paper: https://people.cs.umass.edu/~saisubramanian/GUSSP_iros19_V4.pdf

----------------------------------------------------------------------------------------------------------------


The codebase is built over the MDP framework mdp-lib described in https://github.com/luisenp/mdp-lib. The codebase has been implemented in C++ and tested on Ubuntu 14.04.

Dependencies:

<ul>
    <li>yacc -- In Ubuntu you can run <code>sudo apt-get install bison</code>. </li>
    <li>flex -- In Ubuntu you can run <code>sudo apt-get install flex</code>. </li>
    <li>lsocket -- In Ubuntu you can run <code>sudo apt-get install happycoders-libsocket-dev</code>. </li>
  </ul>

----------------------------------------------------------------------------------------------------------------

Compilation: make testGussp.out

To run experiments:

cd scripts
./runGUSSP.sh


Misc notes:

Three approach to select goal states for determinization:
randomly select goals = 0
select most likely goal (based on current belief) = 1
select closest potential goal = 2




