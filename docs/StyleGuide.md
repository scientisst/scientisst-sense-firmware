# Code Style Guide

This document describes the code style for the project.

## Clang Format

The project uses [Clang Format](https://clang.llvm.org/docs/ClangFormat.html) to format the code. The configuration file
is [.clang-format](../.clang-format) in the root of the project. This file can be used to automatically format the code
before.
It includes automatic formatting based on Microsofts C++ style, identation and include order.

Make sure to run it before committing your code. Most IDEs have a plugin for Clang Format so you can run it
automatically on save.

## Naming Conventions

### File Names

File names should start with `sci_`, be in `snake_case` and end with the corresponding file extension. For
example `sci_my_file.c` or `sci_my_file.h`. This makes it easier to distinguish between files from the ScientISST
project and files from ESP-IDF or other libraries. It also ensures that there are no naming conflicts.

### Variable Names

Variable names should be in `snake_case`. For example `my_variable`. If variables have a unit, the variable name should
end with the unit. For example `my_variable_mV` for a variable with the unit milliVolt.

### Function Names

Function names should be in `camelCasr`. For example `myFunction()`. All ESP-IDF functions are in `snake_case` so this
makes it easier to distinguish between ESP-IDF functions and ScientISST functions.

### Type Names

Type names should be in `snake_case` and end with `_t`. For example `my_enum_t`.

### Macro Names

Macro names should be in `UPPER_SNAKE_CASE`. For example `MY_MACRO`.

## Include Guards

All header files should have include guards. The preferred option is:

```C
#pragma once
```

over the traditional include guards:

```C
#ifndef MY_HEADER_H
#define MY_HEADER_H
// Code
#endif
```

## Function and variable scope

All functions and varibles that are only used inside a file should be declared as `static`. This makes it easier to see
and understand the scope of the function or variable. Furthermore, ensures that there are no naming conflicts and that a
function is not used anywhere it shouldn't.

## Documentation

All files and functions should be documented using Doxygen. This makes it easier to understand the code and generate
documents. The documentation should be written in English and aim to be simple. Functions should be documented using the
following template:

```C
/**
 * \brief This is a brief description of the function.
 *
 * This is a more detailed description of the function.
 *
 *
 * \param[in] var1 A variable that is only read and never modified.
 * \param[in/out] var2 A variable that is read and can be modified. For example a pointer to a buffer that has to be previously allocated.
 * \param[out] var3 A variable that is only modified and never read. For example a pointer to a buffer that will be allocated inside the function.
 *
 * \return SUCCES_MACRO - Succes, ERROR_MACRO1 - Error A, ERROR_MACRO2 - Error B.
 */
```
