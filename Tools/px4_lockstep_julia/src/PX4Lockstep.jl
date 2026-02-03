module PX4Lockstep

using Libdl

export LockstepConfig,
    LockstepCmd,
    LockstepHandle,
    create,
    destroy,
    load_mission,
    step_uorb!,
    set_cmd!,
    param_set_i32,
    param_set_f32,
    param_get_i32,
    param_get_f32,
    param_preinit_set_i32,
    param_preinit_set_f32,
    param_preinit_set!,
    param_set!,
    param_get,
    param_notify!,
    control_alloc_update_params!,
    UORBPublisher,
    UORBSubscriber,
    create_publisher,
    create_subscriber,
    publish!,
    uorb_publisher_instance,
    uorb_topic_metadata,
    uorb_check!,
    uorb_copy!,
    uorb_unsubscribe!,
    verify_uorb_type!,
    verify_uorb_contract!,
    find_library

# Simulation framework lives in a submodule so the core PX4 lockstep wrapper stays small.
export Sim
export Workflows
export Tests

include("lockstep/Library.jl")
include("lockstep/Handle.jl")
include("lockstep/Params.jl")
include("lockstep/UORBContract.jl")

# Generated uORB message types + traits
include("UORBGenerated.jl")

include("lockstep/UORB.jl")

include("sim/Sim.jl")
include("Workflows/Workflows.jl")
include("Tests/Tests.jl")
include("PrecompileWorkload.jl")

end
