/**
 * @file
 * @author  Roland DENIS (\c denis@math.univ-lyon1.fr)
 * @date    22 Feb. 2021
 * @brief   Source file to speedup compilation of the tests that rely on Catch
 *
 * See https://github.com/catchorg/Catch2/blob/devel/docs/slow-compiles.md
 * and https://stackoverflow.com/a/1388969
 */

#define CATCH_CONFIG_MAIN
#include "DGtalCatch.h"
