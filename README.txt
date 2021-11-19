# MDP
Kasey Evans - kevans66

# General Comments
This project contains both Python and Java code (I know -- a true blasphemy).
The grid world problem was run with the Java package BURLAP, while the non-grid world
problem was run with the Python package hiive-mdptoolbox. Data analysis was mostly conducted
in Python.

# Creating a Python environment
conda create -n kevans66-hw4 python=3.9
conda activate kevans66-hw4
conda install swig
pip install -r requirements.txt
git clone https://github.com/hiive/hiivemdptoolbox.git
pip install -e hiivemdptoolbox