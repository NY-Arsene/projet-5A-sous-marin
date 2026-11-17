/** @file mini_tests.h  Mini-framework de test unitaire (un seul en-tête, aucune dépendance). */
#ifndef MINI_TESTS_H
#define MINI_TESTS_H

#include <stdio.h>
#include <math.h>
#include "types_capteurs.h"
#include "materiel_faux.h"

static int mu_tests, mu_failed_tests, mu_checks, mu_cur_failed;

#define MU_ASSERT(cond) do { mu_checks++; if (!(cond)) { \
    printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); mu_cur_failed = 1; } } while (0)

#define MU_ASSERT_INT(expected, actual) do { long long e_ = (long long)(expected), a_ = (long long)(actual); \
    mu_checks++; if (e_ != a_) { printf("    FAIL %s:%d: attendu %lld, obtenu %lld (%s)\n", \
    __FILE__, __LINE__, e_, a_, #actual); mu_cur_failed = 1; } } while (0)

#define MU_ASSERT_NEAR(expected, actual, tol) do { double e_ = (double)(expected), a_ = (double)(actual); \
    mu_checks++; if (!(fabs(e_ - a_) <= (double)(tol))) { printf("    FAIL %s:%d: attendu %.6g +-%.3g, obtenu %.6g (%s)\n", \
    __FILE__, __LINE__, e_, (double)(tol), a_, #actual); mu_cur_failed = 1; } } while (0)

#define MU_ASSERT_STATUS(expected, actual) do { sens_status_t e_ = (expected), a_ = (actual); \
    mu_checks++; if (e_ != a_) { printf("    FAIL %s:%d: attendu %s, obtenu %s\n", \
    __FILE__, __LINE__, sens_status_str(e_), sens_status_str(a_)); mu_cur_failed = 1; } } while (0)

#define MU_RUN(fn) do { fake_reset(); mu_cur_failed = 0; fn(); mu_tests++; \
    if (mu_cur_failed) { mu_failed_tests++; printf("[FAIL] %s\n", #fn); } else { printf("[ ok ] %s\n", #fn); } } while (0)

#define MU_SUMMARY() ( printf("--- %d tests, %d en echec, %d verifications\n", mu_tests, mu_failed_tests, mu_checks), \
                       (mu_failed_tests ? 1 : 0) )

#endif
