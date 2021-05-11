#pragma once

#define ASSERT(PREDICATE, ...)                                          \
  do {                                                                  \
    if (!(PREDICATE)) {                                                 \
      fprintf(stderr,                                                   \
              "%s:%d (%s) Assertion " #PREDICATE " failed: ", __FILE__, \
              __LINE__, __PRETTY_FUNCTION__);                           \
      abort();                                                          \
    }                                                                   \
  } while (0)
