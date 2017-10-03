// Copyright (c) 2017, The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
// * Neither the name of the University of California nor the
//   names of its contributors may be used to endorse or promote products
//   derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "cogrob/cloud/basic/defaults.h"

#include <stdlib.h>
#include <unistd.h>

#include <string>
#include "third_party/gflags.h"

#define AUTO_PLACEHOLDER "__auto__"

DEFINE_string(grpc_roots, "/opt/cogrob/credentials/grpc_roots.pem",
    "gRPC trusted root certificate store path.");

DEFINE_string(gcloud_cred, "/opt/cogrob/credentials/gcloud_credentials.json",
    "Google Cloud APIs credentials json path.");

DEFINE_string(gcloud_project, AUTO_PLACEHOLDER,
    "Google Cloud Platform project name.");

DEFINE_string(agent, AUTO_PLACEHOLDER, "The name of the robot.");

namespace cogrob {
namespace cloud {

void PrepareGoogleCloudCredentials() {
  setenv("GRPC_DEFAULT_SSL_ROOTS_FILE_PATH", FLAGS_grpc_roots.c_str(), 0);
  setenv("GOOGLE_APPLICATION_CREDENTIALS", FLAGS_gcloud_cred.c_str(), 0);
}

std::string GetAgentName() {
  std::string result = FLAGS_agent;
  if (result == AUTO_PLACEHOLDER) {
    const int BUF_LEN = 1024;
    char hostname[BUF_LEN];
    hostname[BUF_LEN - 1] = '\0';
    gethostname(hostname, BUF_LEN - 1);
    result = hostname;
  }
  return result;
}

std::string GetGcloudProjectName() {
  std::string result = FLAGS_gcloud_project;
  if (result == AUTO_PLACEHOLDER) {
    std::string agent_name = GetAgentName();
    if (agent_name.find("fetch") == 0 || agent_name.find("freight") == 0 ||
        agent_name.find("hsr") == 0) {
      result = "cogrob-prod";
    } else {
      result = "cogrob-devel";
    }
  }
  return result;
}

}  // namespace cloud
}  // namespace cogrob
