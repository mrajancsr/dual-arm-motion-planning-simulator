/**
 * Arm type selector component
 */

export function ArmSelector({ value, onChange, disabled }) {
  const armTypes = [
    { id: '2-link', name: '2-Link Arm', dim: '4D' },
    { id: '3-link', name: '3-Link Arm', dim: '6D' },
    { id: '6-link', name: '6-Link Arm', dim: '12D' }
  ];
  
  return (
    <div className="space-y-2">
      <label className="block text-sm font-medium text-gray-300">
        Arm Type
      </label>
      <select
        value={value}
        onChange={(e) => onChange(e.target.value)}
        disabled={disabled}
        className="w-full px-3 py-2 bg-gray-700 text-white rounded border border-gray-600 focus:outline-none focus:border-blue-500 disabled:opacity-50 disabled:cursor-not-allowed"
      >
        {armTypes.map(arm => (
          <option key={arm.id} value={arm.id}>
            {arm.name} ({arm.dim})
          </option>
        ))}
      </select>
    </div>
  );
}

