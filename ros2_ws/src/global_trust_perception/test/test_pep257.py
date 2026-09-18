# # Copyright 2015 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

# from ament_pep257.main import main
# import pytest


# @pytest.mark.linter
# @pytest.mark.pep257
# def test_pep257() -> None:
#     # Project docstring convention: summary on the 2nd line, first line may be a
#     # formula or end with ':' - so ignore the codes that fight that style, and
#     # skip vendored SORT/ (third-party GPL code).
#     rc = main(argv=[
#         '.', 'test',
#         '--exclude', 'global_trust_perception/SORT',
#         '--add-ignore', 'D205', 'D208', 'D209', 'D213', 'D400', 'D401', 'D415',
#     ])
#     assert rc == 0, 'Found code style errors / warnings'
