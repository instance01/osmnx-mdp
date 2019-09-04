#ifndef QUEUE_UTIL_HEADER
#define QUEUE_UTIL_HEADER
#include <vector>
#include <algorithm>

template<typename I, typename P>
I queue_pop(std::vector<std::pair<I, P>> &queue) {
    std::pop_heap(
            queue.begin(),
            queue.end(),
            [](auto &item1, auto &item2) { return item1.second > item2.second; });
    I node = queue.back().first;
    queue.pop_back();
    return node;
}

template<typename I, typename P>
void queue_decrease_priority(std::vector<std::pair<I, P>> &queue, I y, P priority) {
    // Set the priority of item y in the queue.
    // Adds the item into the queue, if it does not exist.
    auto it = std::find_if (
            queue.begin(),
            queue.end(),
            [y](auto &item) { return item.first == y; });

    if (it == queue.end()) {
        queue.push_back({y, priority});
    } else {
        queue[std::distance(queue.begin(), it)] = {it->first, priority};
    }

    std::make_heap(queue.begin(), queue.end());
}
#endif
