#include "research_export.h"
#include <sys/resource.h>
#include <cassert>
#include <chrono>
#include <iostream>

int main(int argc,char** argv) {
  assert(argc==4);
  struct rusage before{},after{};
  getrusage(RUSAGE_SELF,&before);
  const auto start=std::chrono::steady_clock::now();
  ResearchExport writer(argv[1],argv[2],argv[3],2);
  for (int i=0;i<80;++i) {
    ResearchExport::Packet packet;
    packet.metadata={{"sequence",i},{"size",1024*1024}};
    packet.messages.emplace_back(1024*1024,uint8_t(i));
    writer.enqueue(std::move(packet));
  }
  writer.close(); writer.close();
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  getrusage(RUSAGE_SELF,&after);
  assert(elapsed>1.0); // Consumer backpressure reached the producer.
  assert(after.ru_maxrss-before.ru_maxrss<32*1024); // Not an 80 MiB unbounded queue.
  std::cout << "80 immutable packets; explicit clean shutdown; backpressure " << elapsed
            << " s; peak RSS growth " << after.ru_maxrss-before.ru_maxrss << " KiB\n";
}
