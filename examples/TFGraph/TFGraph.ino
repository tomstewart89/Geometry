#include <Geometry.h>

/*
 * TODO
 */

using namespace Geometry;
using namespace BLA;

constexpr static int max_frames = 10;

struct Frame
{
    const char* name;
    int index;
    bool visited = false;

    Frame(const char* name_, int index_) : name(name_), index(index_) {}
};

struct TransformManager
{
    int num_frames = 0;
    Frame* frames[max_frames] = {NULL};
    Transformation* adjacency[max_frames][max_frames] = {NULL};

    Frame& get_frame(const char* frame_id)
    {
        for (int i = 0; i < num_frames; ++i)
        {
            if (strcmp(frames[i]->name, frame_id) == 0)
            {
                return *frames[i];
            }
        }

        frames[num_frames] = new Frame(frame_id, num_frames);

        return *frames[num_frames++];
    }

    /**
     * @brief Adds a connection between a target frame and a source frame such that: P_target = transform * P_source
     *   Where:
     *   * P_source is a point expressed in the source frame
     *   * P_target is the same point expressed in the target frame
     */
    bool add_transform(const char* target_frame_id, const char* source_frame_id, const Transformation& transform)
    {
        if (can_transform(target_frame_id, source_frame_id))
        {
            // There's already a sequence of transformations that'll get us from the target to the source frame
            return false;
        }

        Frame& target_frame = get_frame(target_frame_id);
        Frame& source_frame = get_frame(source_frame_id);

        adjacency[target_frame.index][source_frame.index] = new Transformation(transform);
        adjacency[source_frame.index][target_frame.index] = new Transformation(transform.inv());

        return true;
    }

    bool can_transform(const char* target_frame, const char* source_frame)
    {
        Transformation tmp;
        return get_transform(target_frame, source_frame, tmp);
    }

    /**
     * @brief Gets the transform which maps a point expressed in the source frame into the target frame.
     */
    bool get_transform(const char* target_frame_id, const char* source_frame_id, Transformation& transform)
    {
        for (int i = 0; i < num_frames; ++i)
        {
            frames[i]->visited = false;
        }

        int stack_size = 0;
        Frame* stack[max_frames * max_frames];

        stack[stack_size++] = &get_frame(target_frame_id);
        Frame* source_frame = &get_frame(source_frame_id);

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

            if (adjacency[frame->index][source_frame->index])
            {
                stack[stack_size++] = source_frame;
                break;
            }
            else
            {
                for (int i = 0; i < num_frames; ++i)
                {
                    if (adjacency[frame->index][i])
                    {
                        stack[stack_size++] = frames[i];
                    }
                }
            }
        }

        if (stack_size)
        {
            transform = Identity<4>();
            for (int i = 0; i < stack_size; ++i)
            {
                Serial << frames[i]->name << "\n";
            }
            return true;
        }
        else
        {
            return false;
        }
    }
};

void setup()
{
    Serial.begin(115200);

    TransformManager manager;

    // Part 1: Two frames
    manager.add_transform("A", "B", Transformation(Identity<3>(), {1, 2, 3}));

    Transformation T;
    manager.get_transform("A", "B", T);  // translation of[1, 2, 3]
    manager.get_transform("B", "A", T);  // #translation of[-1, -2, -3]

    //     print("\nPart 2: Four frames chained together")
    //     manager.add_transform("B", "C", translate([3, 2, 1]))
    //     manager.add_transform("C", "D", translate([1, 1, 1]))
    //     print(manager.get_transform("A", "D"))  # translation of [5, 5, 5]
    //     print(manager.get_transform("D", "A"))  # translation of [5,-5,-5]

    //     print("\nPart 3: Draw a square")
    //     translate_then_turn_left = np.eye(4)
    //     translate_then_turn_left[:3, 3] = [1, 0, 0]
    //     translate_then_turn_left[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    //     manager.add_transform("bottom_left", "bottom_right", translate_then_turn_left)
    //     manager.add_transform("bottom_right", "top_right", translate_then_turn_left)
    //     manager.add_transform("top_right", "top_left", translate_then_turn_left)
    //     manager.add_transform("top_left", "back_where_we_started", translate_then_turn_left)

    //     print(manager.get_transform("bottom_left", "back_where_we_started"))  # identity

    //     print("\nPart 4: Frames may have multiple parents and children")
    //     manager.add_transform("0", "B", translate([1, 2, 3]))
    //     manager.add_transform("C", "1", translate([10, 10, 10]))
    //     print(manager.get_transform("0", "1"))  # translation of [14, 14, 14]
    //     print(manager.get_transform("A", "D"))  # translation of [5, 5, 5]

    //     print("\nPart 5: Edge cases")
    //     print(manager.get_transform("A", "A"))  # identity
    //     print(manager.get_transform("A", "idk"))  # None
}

void loop() {}
