// Host stub: the log file class; records which files the handler opens.
#pragma once
class flexible_log_file_implementation_t
{
public:
  bool open (const char *file_name);
  void close (void);
  bool flush_buffer (void) { return true; }
  bool sync_file (void) { return true; }
  void block_input (void) {}
};
