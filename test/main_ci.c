#include "banc_test.h"
#include "materiel.h"
#include <stdio.h>

int main(void) {
    port_init();
    int failures = bench_run_all();
    if (failures > 0) {
        fprintf(stderr, "ECHEC CI : %d test(s) du banc en erreur.\n", failures);
        return 1;
    }
    return 0;
}