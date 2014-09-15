#include <boost/test/unit_test.hpp>
#include <slam3d/Dummy.hpp>

using namespace slam3d;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    slam3d::DummyClass dummy;
    dummy.welcome();
}
