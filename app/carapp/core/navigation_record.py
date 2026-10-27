"""Navigation evidence reuses the existing PC-only recording ownership rules."""
from carapp.core.mapping_record import MappingRecord


class NavigationRecord(MappingRecord):
    def event(self, text, **details):
        super().event(text.replace("建图", "导航"), **details)

