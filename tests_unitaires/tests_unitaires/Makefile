# Tests unitaires sur PC (gcc), lancés depuis ce dossier ou avec : make -C tests_unitaires test
#   make test   : compile et lance les 14 programmes de test
#   make asan   : mêmes tests avec détection de débordements mémoire et de comportements indéfinis
#   make clean  : supprime le dossier construction/
SRC     := ../stm32_sous_marin
CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wshadow -Wpedantic -Werror -g -O0 \
           -I$(SRC)/entetes -I$(SRC)/bibliotheque -I$(SRC)/pilotes -I$(SRC)/application -I$(SRC)/banc_test -I.
LDLIBS  := -lm

LIB_SRC     := $(wildcard $(SRC)/bibliotheque/*.c)
DRIVER_SRC  := $(wildcard $(SRC)/pilotes/*.c)
APP_SRC     := $(wildcard $(SRC)/application/*.c)
BENCH_SRC   := $(SRC)/banc_test/banc_test.c
FAKE_SRC    := materiel_faux.c faux_registres.c faux_composants.c
CORE_SRC    := $(LIB_SRC) $(DRIVER_SRC) $(APP_SRC)

TESTS := $(patsubst %.c,construction/%,$(wildcard test_*.c))

all: test

construction:
	mkdir -p construction

construction/test_banc_test: test_banc_test.c $(CORE_SRC) $(BENCH_SRC) $(FAKE_SRC) | construction
	$(CC) $(CFLAGS) $(SAN) -o $@ $^ $(LDLIBS)

construction/%: %.c $(CORE_SRC) $(FAKE_SRC) | construction
	$(CC) $(CFLAGS) $(SAN) -o $@ $^ $(LDLIBS)

test: $(TESTS)
	@fail=0; for t in $(TESTS); do echo "##### $$t"; ./$$t || fail=1; done; \
	if [ $$fail -ne 0 ]; then echo "ECHEC"; exit 1; else echo "TOUS LES TESTS PASSENT"; fi

asan:
	$(MAKE) clean
	$(MAKE) test SAN="-fsanitize=address,undefined -fno-sanitize-recover=all"

clean:
	rm -rf construction *.gcov
