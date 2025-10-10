# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

require "cgi/escape"
require "json"
require "open-uri"

module Helper
  module_function
  def cmake_lists_txt
    File.join(__dir__, "CMakeLists.txt")
  end

  def detect_version
    File.read(cmake_lists_txt)[/^project\(.+ VERSION (.+?)\)/, 1]
  end

  def update_content(path)
    content = File.read(path)
    content = yield(content)
    File.write(path, content)
  end

  def update_cmake_lists_txt_version(new_version)
    update_content(cmake_lists_txt) do |content|
      content.sub(/^(project\(.* VERSION )(?:.*?)(\))/) do
        "#{$1}#{new_version}#{$2}"
      end
    end
  end

  def github_repository
    ENV["GITHUB_REPOSITORY"] || "enactic/openarm_can"
  end

  def call_github_api(path, **parameters)
    url = "https://api.github.com/#{path}"
    unless parameters.empty?
      encoded_parameters = parameters.collect do |key, value|
        "#{CGI.escape(key.to_s)}=#{CGI.escape(value.to_s)}"
      end
      url += "?#{encoded_parameters.join("&")}"
    end
    URI.open(url) do |response|
      JSON.parse(response.read)
    end
  end

  def wait_github_actions_workflow(branch, workflow_file)
    run_id = nil

    3.times do
      response = call_github_api("repos/#{github_repository}/actions/runs",
                                 branch: branch)
      run = response["workflow_runs"].find do |workflow_run|
        workflow_run["path"] == ".github/workflows/#{workflow_file}"
      end
      if run
        run_id = run["id"]
        break
      end
      sleep(60)
    end
    raise "Couldn't find target workflow" if run_id.nil?

    run_request_path = "repos/#{github_repository}/actions/runs/#{run_id}"
    loop do
      response = call_github_api(run_request_path)
      status = response["status"]
      return if response["status"] == "completed"
      puts("Waiting...: #{status}")
      sleep(60)
    end
  end
end
