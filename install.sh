#!/bin/sh

# download and install the algorithm library
sudo apt install git
git clone https://github.com/comrob/crl.git crl
cd crl
./install.sh
cd ..

# install libraries needed for compiling
sudo apt install -y ccache
sudo apt install libcairo2-dev liblog4cxx-dev libboost-all-dev

# find a CPLEX optimizer
cplex_test=`locate "cplex/bin" | wc -l`
if [ $cplex_test -eq 0 ]
then
  echo -e "\e[31mCPLEX not found!\e[0m"
  exit
fi

cplex_pwd=`locate "cplex/bin" | head -n 1 | sed -e "s/bin//" | sed -e "s/\/cplex//"`
echo "\e[32mCPLEX found\e[0m in directory: ${cplex_pwd}"