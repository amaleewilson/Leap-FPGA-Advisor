#include "fpga_common.h"

using namespace llvm;
using namespace fpga;

void ScheduleVisitor::discover_vertex(TraceGraph_vertex_descriptor v, const TraceGraph &graph) {
	// find the latest finishing parent
	// if no parent, start at 0
	int start = -1;
	TraceGraph_in_edge_iterator ii, ie;
	for (boost::tie(ii, ie) = boost::in_edges(v, graph); ii != ie; ii++) {
		start = std::max(start, graph[boost::source(*ii, graph)].minCycEnd);
	}
	start += 1;

	int end = start;
        BasicBlock *BB =  graph[v].basicblock;

        end += FunctionScheduler::get_basic_block_latency_accelerator(LT, BB);



	//std::cerr << "Schedule vertex: (" << v << ") " << graph[v].basicblock->getName().str() <<
	//			" start: " << start << " end: " << end << "\n";
	(*graph_ref)[v].set_min_start(start);
	(*graph_ref)[v].set_min_end(end);
	(*graph_ref)[v].set_start(start);
	(*graph_ref)[v].set_end(end);

	// keep track of the last cycle as seen by the scheduler
	*lastCycle_ref = std::max(*lastCycle_ref, end);
	//std::cerr << "LastCycle: " << *lastCycle_ref << "\n";
}

