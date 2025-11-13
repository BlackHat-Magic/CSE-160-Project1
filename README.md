# CSE 160

Public because the provided TinyOS image doesn't play nice with basically anything so this is the easiest way to clone the repo onto it.

## Project 1: Objective and Goals

### Flooding

- Each node floods a packet to all neighbor nodes.
- Packets continue to flood until they reach their final destination.
- Must work as ping and ping replies.
- Use only information available from the packet and headers.

### Neighbor Discovery

- Each node should be able to discover all of its neighbors.
- No new packet type than provided.
- Account for neighbors that drop out of the network.

## Project 2: Link State Routing

## Starting

- Use skeleton code.
- Can only send ping packet to neighbor node.
- If send to a node that is not a neighbor, sending fails.
- Ensure packet reaches destination.

## Requirements

- Use debug channel `FLOODING_CHANNEL` from `includes/channels.h`.
- Channel should print a small message whenever a packet is received/sent
    - State location sent from
- For second portion, use `NEIGHBOR_CHANNEL`.
    - Should be capable of telling its neighbors when a packet is issued.