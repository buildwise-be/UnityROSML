# Confirm a pick/place repeat

Before repeating a completed tile or brick pick, show the item-specific removal
prompt. Cancel leaves the review available. Confirm consumes only the review
and completed attempt that opened the prompt. All pick/place Execute controls
use the same confirmation, and TCP repeat keeps its existing flow.

- [x] Test direct Execute without confirmation, stale confirmations, a fresh
  confirmation for each repeat, and the prompt's Yes and No actions.
- [x] Require the ROS `placed_item_confirmation_required` capability on the
  matching successful completion. Send `placed_item_removed: true` only from
  the confirmed repeat path. Bind the prompt to its plan and completed attempt.
- [x] Retain the placed object identity after successful execution. Validate
  the live scene and original start state before replacing the placed geometry
  with the frozen source geometry. Require acknowledgement and the existing
  full preflight before any repeat motion.
- [x] Run the focused Unity workflow and menu tests.

Unity compiled with zero errors. All 306 focused EditMode cases passed.
The ROS repository records the isolated scene and execution replays.
