"""Publish a message to all configured publishers under `key` (if any)."""
function _publish_uorb!(bridge::UORBBridge, key::Symbol, msg)
    entries = get(bridge.pubs, key, nothing)
    entries === nothing && return nothing
    for (pub, _inst) in entries
        _backend_publish!(bridge.backend, bridge.handle, pub, msg)
    end
    return nothing
end
