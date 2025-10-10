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

require "date"

require_relative "helper"

version = ENV["VERSION"] || Helper.detect_version

archive_base_name = "openarm-can-#{version}"
archive_tar_gz = "#{archive_base_name}.tar.gz"
file archive_tar_gz do
  sh("git", "archive", "HEAD",
     "--prefix", "#{archive_base_name}/",
     "--output", archive_tar_gz)
end

desc "Create #{archive_tar_gz}"
task :dist => archive_tar_gz

namespace :release do
  namespace :version do
    desc "Update versions for a new release"
    task :update do
      new_version = ENV["NEW_VERSION"]
      if new_version.nil?
        raise "You must specify NEW_VERSION=..."
      end
      new_release_date = ENV["NEW_RELEASE_DATE"] || Date.today.iso8601
      Helper.update_cmake_lists_txt_version(new_version)
      Helper.update_content("python/meson.build") do |content|
        content.sub(/^(    version: ').*?('.*)$/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      Helper.update_content("python/pyproject.toml") do |content|
        content.sub(/^(version = ").*?(")$/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      Helper.update_content("python/openarm/can/__init__.py") do |content|
        content.sub(/^(__version__ = ").*?(")$/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      ruby("-C",
           "packages",
           "-S",
           "rake",
           "version:update",
           "RELEASE_DATE=#{new_release_date}")
      sh("git",
         "add",
         "CMakeLists.txt",
         "packages/debian/changelog",
         "packages/fedora/openarm-can.spec",
         "python/meson.build",
         "python/pyproject.toml",
         "python/openarm/can/__init__.py")
      sh("git",
         "commit",
         "-m",
         "Update version info to #{version} (#{new_release_date})")
      sh("git", "push")
    end
  end

  desc "Tag"
  task :tag do
    current_version = Helper.detect_version

    changelog = "packages/debian/changelog"
    case File.readlines(changelog)[0]
    when /\((.+)-1\)/
      package_version = $1
      unless package_version == current_version
        raise "package version isn't updated: #{package_version}"
      end
    else
      raise "failed to detect deb package version: #{changelog}"
    end

    sh("git",
       "tag",
       current_version,
       "-a",
       "-m",
       "OpenArm CAN #{current_version}!!!")
    sh("git", "push", "origin", current_version)
  end

  desc "Release packages for Ubuntu"
  task :ubuntu do
    current_version = Helper.detect_version
    Helper.wait_github_actions_workflow(current_version, "release.yaml")
    ruby("-C",
         "packages",
         "-S",
         "rake",
         "ubuntu")
  end
end

desc "Release"
task release: [
  "release:version:update",
  "release:tag",
  "release:ubuntu",
]
