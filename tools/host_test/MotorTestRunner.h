#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace MotorHostTest
{

struct TestCase
{
    const char* name;
    void (*fn)();
};

inline TestCase* registry()
{
    static TestCase cases[128] = {};
    return cases;
}

inline int& registryCount()
{
    static int count = 0;
    return count;
}

class Registrar
{
public:
    Registrar(const char* name, void (*fn)())
    {
        const int index = registryCount();
        if (index >= 128)
        {
            std::fprintf(stderr, "Too many host tests\n");
            std::abort();
        }
        registry()[index] = {name, fn};
        registryCount() = index + 1;
    }
};

inline void fail(const char* expr, const char* file, int line)
{
    std::fprintf(stderr, "ASSERT failed: %s (%s:%d)\n", expr, file, line);
    std::abort();
}

inline void assertNear(float actual, float expected, float tolerance,
                       const char* expr, const char* file, int line)
{
    if (std::fabs(actual - expected) > tolerance)
    {
        std::fprintf(stderr,
                     "ASSERT_NEAR failed: %s actual=%f expected=%f tolerance=%f (%s:%d)\n",
                     expr, actual, expected, tolerance, file, line);
        std::abort();
    }
}

inline int runAll()
{
    int passed = 0;
    for (int i = 0; i < registryCount(); ++i)
    {
        std::printf("[ RUN      ] %s\n", registry()[i].name);
        registry()[i].fn();
        std::printf("[       OK ] %s\n", registry()[i].name);
        ++passed;
    }
    std::printf("[==========] %d tests passed\n", passed);
    return 0;
}

} // namespace MotorHostTest

#define MOTOR_TEST(name) \
    static void name(); \
    namespace { MotorHostTest::Registrar registrar_##name(#name, &name); } \
    static void name()

#define MOTOR_ASSERT_TRUE(expr) \
    do { if (!(expr)) { MotorHostTest::fail(#expr, __FILE__, __LINE__); } } while (0)

#define MOTOR_ASSERT_EQ(actual, expected) \
    MOTOR_ASSERT_TRUE((actual) == (expected))

#define MOTOR_ASSERT_NEAR(actual, expected, tolerance) \
    MotorHostTest::assertNear((actual), (expected), (tolerance), \
                              #actual " ~= " #expected, __FILE__, __LINE__)
