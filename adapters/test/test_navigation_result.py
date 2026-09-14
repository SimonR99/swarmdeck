from types import SimpleNamespace as NS

import pytest

from adapters.exploration import is_physical_no_progress_failure
from adapters.navigation_result import navigation_failure_reason


@pytest.mark.parametrize("code", [4, 105])
def test_typed_progress_failure_with_empty_message_enters_bounded_recovery(code):
    result = NS(error_msg="", error_code=code, FAILED_TO_MAKE_PROGRESS=code)
    reason = navigation_failure_reason(result)
    assert is_physical_no_progress_failure(reason)
    assert reason == f"Failed to make progress; error_code={code}"


@pytest.mark.parametrize(
    "result",
    [
        NS(
            error_msg="Transform unavailable",
            error_code=102,
            FAILED_TO_MAKE_PROGRESS=105,
        ),
        NS(error_msg="", error_code=105),
        NS(error_code=0, FAILED_TO_MAKE_PROGRESS=0),
        None,
    ],
)
def test_other_results_do_not_claim_physical_stall(result):
    assert not is_physical_no_progress_failure(navigation_failure_reason(result))


def test_existing_diagnostic_is_preserved_without_duplication():
    result = NS(
        error_msg="Failed to make progress",
        message="Failed to make progress",
        error_code=105,
        FAILED_TO_MAKE_PROGRESS=105,
    )
    assert (
        navigation_failure_reason(result) == "Failed to make progress; error_code=105"
    )
