"""Select exact binary transport only for its separately named socket."""
def prove(lc,rc,pairs,socketcfg):
    if str(socketcfg.get('socket','')).endswith('pair-binary.sock'):
        from gpu_pair_schedule_binary import prove as implementation
    else:
        from gpu_pair_schedule_json import prove as implementation
    return implementation(lc,rc,pairs,socketcfg)
