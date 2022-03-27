#include "physics_item.hpp"


static Physics::item_id_t current_item_id = 0;

Physics::item_id_t Physics::generate_item_id() {
    return current_item_id++;
}

