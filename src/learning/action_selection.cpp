#include <learning/action_selection.h>

/** Prints the values of the distribution to terminal.
 */
void
print_distribution(const Action_Selection_Base::Vector_t& distribution)
{
    sts_msg("Length: %u", distribution.size());
    for (std::size_t i = 0; i < distribution.size(); ++i)
        printf("%+1.3f ", distribution[i]);
    printf("\n");
}

/** Selects an index from given discrete probability distribution.
 *  The given distribution must sum up to 1.
 */
std::size_t
Action_Selection_Base::select_from_distribution(const Action_Selection_Base::Vector_t& distribution)
{
    const double x = random_value(0.0, 1.0);
    double sum = 0.0;
    for (std::size_t i = 0; i < distribution.size(); ++i)
    {
        assert((0.0 <= distribution[i]) and (distribution[i] <= 1.0));
        sum += distribution[i];
        if (x < sum)
            return i;
        assert(sum < 1.0);
    }
    dbg_msg("sum: %1.4f", sum);
    assert(false);
}
