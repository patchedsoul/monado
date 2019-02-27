#include <check.h>
#include <stdlib.h>

Suite *system_test_suite(void);
Suite *unit_test_suite(void);

int main(void) {
  Suite *systemtest_suite = system_test_suite();

  SRunner *sr = srunner_create(systemtest_suite);

  Suite *unittest_suite = unit_test_suite();
  srunner_add_suite(sr, unittest_suite);

  srunner_run_all(sr, CK_NORMAL);
  int number_failed = srunner_ntests_failed(sr);
  srunner_free(sr);
  return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
