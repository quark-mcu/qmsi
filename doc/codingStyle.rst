Good QMSI coding practice
#########################

Overview
********

This page provides details on good coding rules and practices for the QMSI
project.

.. contents::


Writing space efficient C code
******************************

The following is a list of suggestions to reduce the size of C code

* Avoid using variables when they are not needed, for example temporary
  variables.
* Use #defines if possible.
* Be selective with the type of size of variables used as there can be a lot of
  variation in their size.
* Using 32 bit integer variables will result in smaller code.
* Avoid using unnecessary branches where possible as they can significantly
  increase the size of the compiled binary....?

http://www.codeproject.com/Articles/6154/Writing-Efficient-C-and-C-Code-Optimization

When to use parenthesis
***********************

==========================      ===========     ===============
Operation                       Parenthesis     Sample
==========================      ===========     ===============
Assignment                      no              A = B
Unary operation                 no              A++
single binary   operation       no              A = B ◊ C
multiple binary operations      yes             A ◊= (B ◊ C)
ternary operation               yes             (A ◊ B) ? C : D
==========================      ===========     ===============

Formatting your code using Clang-format
***************************************

Before submitting your code for review, it is a project requirement that you
format it using clang-format and the project rules list.

clang-format is installed on the iLab server. As such, before you sublit
your code, you can run the following command to ensure that it is formatted
correctly.

**clang-format -style=file -i main.c**

"-style=file" This instructs clang-format to search up through the directory
tree for a ".clang-format" rules file. This can be found in the root directory
of the the Quark Microcontroller BSP directory.

"-i" This instructs clang-format to format the file "inline".

"main.c" This is the name of the file that you with the format.
