__all__ = ["mutation2perturb" , "perturb2mutation", "patch2ca"]

from swagger_server.models.error import Error  # noqa: E501
from swagger_server.models.inline_response200 import InlineResponse200  # noqa: E501
from swagger_server.models.inline_response2001 import InlineResponse2001  # noqa: E501
from swagger_server.models.inline_response2001_resourceconsumption import InlineResponse2001Resourceconsumption  # noqa: E501
from swagger_server.models.parameters import Parameters  # noqa: E501
from swagger_server.models.perturbation import Perturbation  # noqa: E501
from swagger_server.models.perturbation_params import PerturbationParams  # noqa: E501
from swagger_server.models.source_line import SourceLine  # noqa: E501
from swagger_server.models.compilation_outcome import CompilationOutcome
from swagger_server.models.candidate_adaptation import CandidateAdaptation
from swagger_server.models.test_outcome import TestOutcome
from swagger_server.models.source_range import SourceRange
from swagger_server.models.source_location import SourceLocation

import hulk
import darjeeling

def patch2ca(pa):
    outcome_compilation = CompilationOutcome(time_taken=pa.build.time_taken,
                                             successful=pa.build.successful)
    outcome_tests = [
        TestOutcome(test_id=test_id,
                    time_taken=pa.tests[test_id].time_taken,
                    passed=pa.tests[test_id].passed)
        for test_id in pa.tests
    ]
    return CandidateAdaptation(diff=pa.diff,
                               compilation_outcome=outcome_compilation,
                               test_outcomes=outcome_tests)

def perturb2mutation(pp):
    loc = hulk.FileLocationRange(pp.at.start.file,
                                 hulk.Location(pp.at.start.line, pp.at.start.column),
                                 hulk.Location(pp.at.stop.line, pp.at.stop.column))
    return hulk.Mutation(pp.kind,                 # operator
                         pp.transformation_index, # transformation_index,
                         loc,                     # location,
                         pp.arguments)            # args

def mutation2perturb(m):
    return Perturbation(kind = m.operator,
                        at = SourceRange(start=SourceLocation(file=m.location.filename,
                                                              line=m.location.start.line,
                                                              column=m.location.start.column) ,
                                         stop=SourceLocation(file=m.location.filename,
                                                              line=m.location.stop.line,
                                                              column=m.location.stop.column)),
                        replacement = "todo", ## todo
                        transformation_index = m.transformation_index,
                        arguments = m.arguments)
