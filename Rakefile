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

def git_clone_archive(url, tag, archive_path)
  mkdir_p(File.dirname(archive_path))
  clone_dir = File.basename(archive_path, ".tar.gz")
  rm_rf(clone_dir)
  sh("git", "clone", url, "--branch", tag, "--recursive", clone_dir)
  yield(clone_dir) if block_given?
  sh("tar",
     "--exclude-vcs",
     "--exclude-vcs-ignores",
     "-czf", archive_path,
     clone_dir)
  rm_rf(clone_dir)
end

cmakelists = File.read(File.join("python", "CMakeLists.txt"))
nanobind_version = cmakelists[/set\(NANOBIND_BUNDLED_VERSION \"(.+)"\)/, 1]
nanobind_tar_gz = File.join("python", "vendor", "nanobind-#{nanobind_version}.tar.gz")
file nanobind_tar_gz do
  git_clone_archive("https://github.com/wjakob/nanobind.git",
                    "v#{nanobind_version}",
                    nanobind_tar_gz) do |clone_dir|
    rm_rf(File.join(clone_dir, "docs")) # 1.1MB
  end
end

namespace :vendor do
  desc "Download vendored dependencies"
  task download: [
    nanobind_tar_gz,
  ]
end

archive_base_name = "openarm-can-#{version}"
archive_tar_gz = "#{archive_base_name}.tar.gz"
file archive_tar_gz => ["vendor:download"] do
  sh("git", "archive", "HEAD",
     # This --prefix is for --add-file
     "--prefix", "#{archive_base_name}/python/vendor/",
     "--add-file", nanobind_tar_gz,
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
      Helper.update_content("package.xml") do |content|
        content.sub(/(<version>).*?(<\/version>)/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      Helper.update_content("python/CMakeLists.txt") do |content|
        content.sub(/^(  VERSION ).*?$/) do
          "#{$1}#{new_version}"
        end
      end
      Helper.update_content("python/openarm_can/__init__.py") do |content|
        content.sub(/^(__version__ = ").*?(")$/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      Helper.update_content("python/pyproject.toml") do |content|
        content.sub(/^(version = ").*?(")$/) do
          "#{$1}#{new_version}#{$2}"
        end
      end
      Helper.update_content("python/setup.py") do |content|
        content.sub(/^(    version=").*?(",)$/) do
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
         "package.xml",
         "packages/debian/changelog",
         "packages/fedora/openarm-can.spec",
         "python/CMakeLists.txt",
         "python/openarm_can/__init__.py",
         "python/pyproject.toml",
         "python/setup.py")
      sh("git",
         "commit",
         "-m",
         "Update version info to #{new_version} (#{new_release_date})")
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
    Helper.wait_github_actions_workflow(current_version, "package.yaml")
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
