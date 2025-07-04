set(DGTAL_TESTS_SRC_MATH
       testSimpleRegression
       testMeasure
       testSignal
       testStatistics
       testHistogram
       testMPolynomial
       testAngleLinearMinimizer
       testBasicMathFunctions
       testMultiStatistics
       testProfile
       testMeaningfulScaleAnalysis
       testLagrangeInterpolation
       )

if (DGTAL_WITH_FFTW3)
  set (DGTAL_TESTS_SRC_MATH ${DGTAL_TESTS_SRC_MATH} testRealFFT)
endif()


foreach(FILE ${DGTAL_TESTS_SRC_MATH})
  DGtal_add_test(${FILE})
endforeach()

add_subdirectory(linalg)
