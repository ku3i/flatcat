#ifndef STATIC_VECTOR_H_INCLUDED
#define STATIC_VECTOR_H_INCLUDED

#include <algorithm>
#include <vector>
#include <cassert>
#include <common/log_messages.h>

/** this vector wrapper class prevents direct access to the underlying vector.
 *  no copying of elements allowed
 *  constructed via emplace back
 *  no resizing
 *  save indexing
 */

template <typename element_t>
class static_vector {
//TODO    static_vector(const static_vector& other) = delete;      // non construction-copyable
    static_vector& operator=(const static_vector&) = delete; // non copyable
public:

    template<typename... Args>
    explicit static_vector(std::size_t number_of_elements, const Args&... args) : content() {
        content.reserve(number_of_elements);
        for (std::size_t index = 0; index < number_of_elements; ++index)
            content.emplace_back(/*index, */args...);
    }

    virtual ~static_vector() = default;

    std::size_t size() const { return content.size(); }

          element_t& operator[] (std::size_t index)       { assert(index < content.size()); return content[index]; }
    const element_t& operator[] (std::size_t index) const { assert(index < content.size()); return content[index]; }

    element_t   get_max   (void) const { return *std::max_element(content.begin(), content.end()); }
    element_t   get_min   (void) const { return *std::min_element(content.begin(), content.end()); }
    std::size_t get_argmax(void) const { return  std::distance(content.begin(), std::max_element(content.begin(), content.end())); }
    std::size_t get_argmin(void) const { return  std::distance(content.begin(), std::min_element(content.begin(), content.end())); }

protected:
    std::vector<element_t> content;
};

template <typename element_t>
class copyable_static_vector : public static_vector<element_t>
{
public:
    template<typename... Args>
    explicit copyable_static_vector(const Args&... args)
    : static_vector<element_t>(args...)
    {}

    copyable_static_vector& operator=(const copyable_static_vector& other) {
        assert(this->content.size() == other.content.size());
        this->content = other.content;
        return *this;
    }
};



#endif // STATIC_VECTOR_H_INCLUDED
