#pragma once

#include <cctype>
#include <set>
#include <string>

namespace ur16e_mtc_gpio_stage
{
struct RepeatExecutionRequest
{
  std::string plan_id;
  std::string attempt_id;
};

inline bool validExecutionToken(const std::string& token)
{
  if (token.empty() || token.size() > 128) return false;
  for (unsigned char ch : token)
    if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.' && ch != ':') return false;
  return true;
}

// Deliberately narrow JSON contract: exactly two token strings, no nested data,
// escapes, duplicate fields or unknown fields. JsonUtility GUID payloads fit it.
inline bool parseRepeatExecutionRequest(const std::string& json, RepeatExecutionRequest& request)
{
  request = {};
  if (json.size() > 512) return false;
  std::size_t index = 0;
  const auto skip = [&]() { while (index < json.size() && std::isspace(static_cast<unsigned char>(json[index]))) ++index; };
  const auto take = [&](char ch) { skip(); if (index >= json.size() || json[index] != ch) return false; ++index; return true; };
  const auto token = [&](std::string& result) {
    if (!take('"')) return false;
    const auto begin = index;
    while (index < json.size() && json[index] != '"') ++index;
    if (index == json.size()) return false;
    result = json.substr(begin, index++ - begin);
    return validExecutionToken(result);
  };
  if (!take('{')) return false;
  bool plan = false, attempt = false;
  for (int field = 0; field < 2; ++field)
  {
    std::string key, value;
    if (!token(key) || !take(':') || !token(value)) return false;
    if (key == "plan_id" && !plan) { request.plan_id = value; plan = true; }
    else if (key == "attempt_id" && !attempt) { request.attempt_id = value; attempt = true; }
    else return false;
    if (field == 0 && !take(',')) return false;
  }
  if (!take('}')) return false;
  skip();
  return plan && attempt && index == json.size();
}

// The caller holds the task mutex. Original legacy Execute remains one-shot;
// this additional admission requires an explicitly identified completed plan.
class RepeatExecutionGate
{
public:
  void retainSuccessful(const std::string& plan_id)
  {
    plan_id_ = plan_id;
    completed_ = validExecutionToken(plan_id);
    attempts_.clear();
  }
  void invalidate() { plan_id_.clear(); completed_ = false; attempts_.clear(); }
  bool repeatable() const { return completed_; }
  bool isDuplicate(const RepeatExecutionRequest& request) const
  {
    return request.plan_id == plan_id_ && attempts_.count(request.attempt_id) != 0;
  }
  std::string begin(const RepeatExecutionRequest& request)
  {
    if (!validExecutionToken(request.plan_id) || !validExecutionToken(request.attempt_id)) return "Invalid repeat request identifiers";
    if (request.plan_id != plan_id_) return "Repeat plan ID does not match the retained successful plan";
    if (isDuplicate(request)) return "Duplicate repeat attempt ignored";
    if (!completed_) return "The retained plan has no confirmed successful completion";
    // Never evict IDs: forgetting a token could admit a delayed duplicate.
    if (attempts_.size() >= 1024) return "Repeat attempt limit reached; request a new plan";
    attempts_.insert(request.attempt_id);
    completed_ = false;
    return {};
  }
  void finish(bool success) { completed_ = success && !plan_id_.empty(); }

private:
  std::string plan_id_;
  bool completed_ = false;
  std::set<std::string> attempts_;
};
}  // namespace ur16e_mtc_gpio_stage
