<?php
namespace App\Network;
final class OriginPolicy
{
    public function normalize(string $origin): string
    {
        $parts=parse_url(trim($origin));
        if (($parts['scheme']??'')!=='https' || empty($parts['host']) || isset($parts['user']) || isset($parts['pass']) || isset($parts['query']) || isset($parts['fragment']) || ($parts['path']??'')!=='') { throw new \InvalidArgumentException('Une origin HTTPS sans chemin est requise.'); }
        $host=strtolower((string)$parts['host']);
        if ($host==='localhost' || str_ends_with($host,'.local') || filter_var($host,FILTER_VALIDATE_IP)) { throw new \InvalidArgumentException('Origin locale ou IP refusée.'); }
        return 'https://'.$host.(isset($parts['port'])?':'.(int)$parts['port']:'');
    }
}
