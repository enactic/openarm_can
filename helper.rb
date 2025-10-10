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
end
