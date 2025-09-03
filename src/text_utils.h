// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once
#include <string>
#include <cstring>

// Replace all occurrences of `from` with `to` in-place.
inline void rep_all(std::string &s, const char *from, const char *to) {
  const size_t flen = std::strlen(from);
  const size_t tlen = std::strlen(to);
  if (flen == 0) return;
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, flen, to);
    pos += tlen;
  }
}

// ASCII-normalize common Unicode punctuation/symbols/accents we see in media metadata.
// Uses explicit UTF-8 byte sequences so it compiles cleanly under C++17.
inline std::string normalize_media_text(std::string s) {
  // Curly quotes / apostrophes
  rep_all(s, "\xE2\x80\x99", "'"); // ’
  rep_all(s, "\xE2\x80\x98", "'"); // ‘
  rep_all(s, "\xCA\xBC",     "'"); // ʼ

  // Double quotes
  rep_all(s, "\xE2\x80\x9C", "\""); // “
  rep_all(s, "\xE2\x80\x9D", "\""); // ”

  // Dashes & ellipsis
  rep_all(s, "\xE2\x80\x93", "-");   // –
  rep_all(s, "\xE2\x80\x94", "-");   // —
  rep_all(s, "\xE2\x80\xA6", "..."); // …

  // Non-breaking space
  rep_all(s, "\xC2\xA0", " "); // NBSP -> space

  // Symbols we usually drop
  rep_all(s, "\xE2\x84\xA2", ""); // ™
  rep_all(s, "\xC2\xAE",    ""); // ®

  // Very common Latin letters
  rep_all(s, "\xC3\xA9","e"); rep_all(s, "\xC3\xA8","e"); rep_all(s, "\xC3\xAA","e"); rep_all(s, "\xC3\xAB","e"); // é è ê ë
  rep_all(s, "\xC3\xA1","a"); rep_all(s, "\xC3\xA0","a"); rep_all(s, "\xC3\xA4","a"); rep_all(s, "\xC3\xA2","a"); // á à ä â
  rep_all(s, "\xC3\xAD","i"); rep_all(s, "\xC3\xAC","i"); rep_all(s, "\xC3\xAF","i"); rep_all(s, "\xC3\xAE","i"); // í ì ï î
  rep_all(s, "\xC3\xB3","o"); rep_all(s, "\xC3\xB2","o"); rep_all(s, "\xC3\xB6","o"); rep_all(s, "\xC3\xB4","o"); rep_all(s, "\xC3\xB8","o"); // ó ò ö ô ø
  rep_all(s, "\xC3\xBA","u"); rep_all(s, "\xC3\xB9","u"); rep_all(s, "\xC3\xBC","u"); rep_all(s, "\xC3\xBB","u"); // ú ù ü û
  rep_all(s, "\xC3\xB1","n"); // ñ
  rep_all(s, "\xC3\xA7","c"); // ç
  rep_all(s, "\xC3\x9F","ss"); // ß

  return s;
}

// Build absolute URL from HA path (or pass through absolute URLs).
inline std::string normalize_ha_image_url(const std::string &in, const std::string &base) {
  if (in.rfind("http://", 0) == 0 || in.rfind("https://", 0) == 0) return in;
  if (in.empty()) return base;
  const bool base_has_slash = !base.empty() && base.back() == '/';
  const bool path_has_slash = !in.empty() && in.front() == '/';
  if (base_has_slash && path_has_slash) return base.substr(0, base.size() - 1) + in;
  if (!base_has_slash && !path_has_slash) return base + "/" + in;
  return base + in;
}
