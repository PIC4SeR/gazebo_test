from launch_pal.arg_utils import LaunchArgumentsBase
from dataclasses import dataclass
from launch.substitutions import LaunchConfiguration


@dataclass(frozen=True, kw_only=True)
class LaunchArgumentsBaseParam(LaunchArgumentsBase):

    def launch_configurations(self) -> list[LaunchConfiguration]:
        """Return a list of launch configurations for the launch file
        Returns:
            A list of launch configurations.
        """

        return [
            LaunchConfiguration(f"{name}") for name in self.__dataclass_fields__.keys()
        ]

    def launch_configurations_dict(self) -> dict[str, LaunchConfiguration]:
        """Return a dictionary of launch configurations."""
        return {
            name: LaunchConfiguration(f"{name}")
            for name in self.__dataclass_fields__.keys()
        }

    def launch_configurations_dict_with_context(
        self,
        context=None,
    ) -> dict[str, str]:
        """Return a dictionary of launch configurations with context."""
        if context is None:
            raise ValueError(
                "Context cannot be None, please use this function in a OpaqueFunction"
            )
        return {
            name: config.perform(context)
            for name, config in self.launch_configurations_dict().items()
        }

    def param_rewrites_dict(self, context=None) -> dict[str, str]:
        """Return a dictionary of parameter rewrites."""
        if context is None:
            raise ValueError(
                "Context cannot be None, please use this function in a OpaqueFunction"
            )
        return {
            name: config
            for name, config in self.launch_configurations_dict_with_context(
                context
            ).items()
            if config
        }
