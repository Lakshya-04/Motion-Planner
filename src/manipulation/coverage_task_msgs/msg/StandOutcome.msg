# What happened at one stand of a multi-stand run (one entry per planned stand, in visiting order).
uint32 stand_index
# false when the base could not be driven to the stand (after the retries): its columns were not run.
bool reached
string message
# Columns are indices into the loaded coverage plan (the order of the file).
uint32[] columns_done
uint32[] columns_partial     # some, not all, of the column's poses were completed
uint32[] columns_failed
