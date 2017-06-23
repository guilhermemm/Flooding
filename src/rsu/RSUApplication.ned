//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package flooding.rsu;

import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

//
// TODO auto-generated module
//
simple RSUApplication extends BaseWaveApplLayer
{
    parameters:           
        // logging parameters
        int log_traffic;
        int log_replication;
                                
    	double startDataProductionTime @unit(s) = default(20s);    	
    	double dataROI @unit(m) = default(5000m);
    	double dataTTL @unit(s) = default(60s);
    	
    	int numberPackets = default(100);
    	int packetSize @unit(bit) = default(16384bit);
    	
    	double datarate @unit(bps) = default(2Mbps);
        
    	@class(RSUApplication);
}
