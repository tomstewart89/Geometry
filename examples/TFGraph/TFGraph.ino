#include <Geometry.h>

/*
 * TODO
 */

using namespace Geometry;
using namespace BLA;

struct Frame
{
    const char* name;
    int index;
    bool visited;
    Frame* parent;

    Frame(const char* name_, int index_) : name(name_), index(index_) {}
};

struct TransformManager
{
    constexpr static int max_frames = 50;

    int num_frames = 0;
    Frame* frames[max_frames];
    Transformation* adjacency[max_frames][max_frames] = {NULL};

    ~TransformManager();

    Frame* get_frame(const char* frame_id);

    bool add_transform(const char* target_frame_id, const char* source_frame_id, const Transformation& transform);

    bool get_transform(const char* target_frame_id, const char* source_frame_id, Transformation& transform);
};

TransformManager::~TransformManager()
{
    for (int i = 0; i < max_frames; ++i)
    {
        if (frames[i])
        {
            delete frames[i];
        }
        for (int j = 0; j < max_frames; ++j)
        {
            if (adjacency[i][j])
            {
                delete adjacency[i][j];
            }
        }
    }
}

Frame* TransformManager::get_frame(const char* frame_id)
{
    for (int i = 0; i < num_frames; ++i)
    {
        if (strcmp(frames[i]->name, frame_id) == 0)
        {
            return frames[i];
        }
    }

    frames[num_frames] = new Frame(frame_id, num_frames);

    return frames[num_frames++];
}

bool TransformManager::add_transform(const char* target_frame_id, const char* source_frame_id,
                                     const Transformation& transform)
{
    Transformation tmp;
    if (get_transform(target_frame_id, source_frame_id, tmp))
    {
        return false;
    }

    Frame* target_frame = get_frame(target_frame_id);
    Frame* source_frame = get_frame(source_frame_id);

    adjacency[target_frame->index][source_frame->index] = new Transformation(transform);
    adjacency[source_frame->index][target_frame->index] = new Transformation(transform.inv());

    return true;
}

bool TransformManager::get_transform(const char* target_frame_id, const char* source_frame_id,
                                     Transformation& transform)
{
    for (int i = 0; i < num_frames; ++i)
    {
        frames[i]->visited = false;
    }

    Frame* target_frame = get_frame(target_frame_id);
    Frame* source_frame = get_frame(source_frame_id);

    Frame* stack[max_frames * max_frames];

    int stack_size = 0;
    stack[stack_size++] = target_frame;

    while (stack_size)
    {
        Frame* frame = stack[--stack_size];

        if (frame->visited)
        {
            continue;
        }
        else
        {
            frame->visited = true;
        }

        for (int i = 0; i < num_frames; ++i)
        {
            if (adjacency[frame->index][i] && !frames[i]->visited)
            {
                stack[stack_size++] = frames[i];
                frames[i]->parent = frame;

                if (frames[i] == source_frame)
                {
                    goto found_source_frame;
                }
            }
        }
    }

found_source_frame:

    if (!stack_size)
    {
        return false;
    }

    Transformation T_source_target(Identity<3>(), {0, 0, 0});

    Frame* frame = source_frame;

    while (true)
    {
        if (frame == target_frame)
        {
            break;
        }

        T_source_target = *adjacency[frame->index][frame->parent->index] * T_source_target;
        frame = frame->parent;
    }

    transform = T_source_target.inv();
    return true;
}

void setup()
{
    Serial.begin(115200);

    TransformManager manager;

    manager.add_transform("A", "B", {Identity<3>(), {1, 0, 0}});
    manager.add_transform("B", "C", {Identity<3>(), {0, 1, 0}});
    manager.add_transform("C", "D", {Identity<3>(), {0, 0, 1}});

    Transformation T_AD;
    manager.get_transform("A", "D", T_AD);
    Serial << "D frame measured from A:\n" << T_AD << "\n";

    manager.add_transform("B", "E", {Identity<3>(), {1, 2, 3}});
    manager.add_transform("E", "D", {Identity<3>(), {1, 2, 3}});

    Transformation translate_then_turn_left({0, -1, 0, 1, 0, 0, 0, 0, 1}, {0, 1, 0});

    manager.add_transform("bottom_left", "bottom_right", translate_then_turn_left);
    manager.add_transform("bottom_right", "top_right", translate_then_turn_left);
    manager.add_transform("top_right", "top_left", translate_then_turn_left);
    manager.add_transform("top_left", "back_where_we_started", translate_then_turn_left);

    manager.get_transform("bottom_left", "back_where_we_started", T_AD);  //#identity

    Serial << T_AD;

    Transformation T_AA;
    manager.get_transform("A", "A", T_AA);
    Serial << "T_AA:\n" << T_AA << "\n";
}

void loop() {}
